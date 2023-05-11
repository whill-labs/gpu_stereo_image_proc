#!/usr/bin/env python

#
# Helper script for testing image processing nodes
#   Allows specification of one or more "input" image and camera_info **files**
#   on the command line.  It then publishes those topics **once**
#   and waits for a number of "output" image topics, saving the first
#   received result from each to a file (or timeout).
#
# Args use the format "{ROS topic}:{filename}"
#
# Note this script **does not** do any image processing (e.g. rectification)
#
# Sample usage would be something like:
#
#  ./image_oneshot.py \
#        --input-image /left/image_raw:left_image.png \
#        --camera-info /left/camera_info:left_calibration.yaml \
#        --input-image /right/image_raw:right_image.png \
#        --camera-info /right/camera_info:right_calibration.yaml \
#        --output-disparity /disparity/image:disparity.png
#


import os.path
import cv2
import yaml
import argparse
import rospy
from cv_bridge import CvBridge
import numpy as np

from sensor_msgs.msg import CameraInfo, Image


def yaml_to_np(val):
    return np.array(val["data"]).reshape(val["cols"], val["rows"])


class InputImagePub:
    def __init__(self, arg):

        topic, filename = arg.split(":")

        if not os.path.exists(filename):
            raise IOError('Can\'t find input image "%s"' % filename)

        bridge = CvBridge()
        cv_image = cv2.imread(filename)

        if cv_image.size == 0:
            raise IOError('Couldn\'t load input image "%s"' % filename)

        self.ros_image = bridge.cv2_to_imgmsg(cv_image, "rgb8")

        rospy.loginfo('Publishing image "%s" on topic %s' % (filename, topic))
        self.pub = rospy.Publisher(topic, Image, queue_size=1)

    def publish(self, ts):
        self.ros_image.header.stamp = ts
        rospy.loginfo("Publishing info")
        self.pub.publish(self.ros_image)


class CameraInfoPub:
    def __init__(self, arg):

        topic, filename = arg.split(":")

        self.msg = CameraInfo()

        with open(filename) as fp:
            camera_params = yaml.safe_load(fp)

        self.msg.height = camera_params["image_height"]
        self.msg.width = camera_params["image_width"]

        # Rectify incoming images
        self.msg.K = yaml_to_np(camera_params["camera_matrix"]).flatten().tolist()
        self.msg.D = (
            yaml_to_np(camera_params["distortion_coefficients"]).flatten().tolist()
        )
        self.msg.R = (
            yaml_to_np(camera_params["rectification_matrix"]).flatten().tolist()
        )
        self.msg.P = yaml_to_np(camera_params["projection_matrix"]).flatten().tolist()
        self.msg.distortion_model = camera_params["camera_model"]

        rospy.loginfo('Publishing camera_info "%s" on topic %s' % (filename, topic))

        rospy.loginfo(self.msg.D)
        self.pub = rospy.Publisher(topic, CameraInfo, queue_size=1)

    def publish(self, ts):
        self.msg.header.stamp = ts
        rospy.loginfo("Publishing camera info")
        self.pub.publish(self.msg)


class OutputImageSub:
    def __init__(self, arg):
        self.captured = False

        args = arg.split(":")

        if len(args) == 2:
            self.topic, self.filename = args
            self.encoding = "passthrough"
        elif len(args) == 3:
            self.topic, self.filename, self.encoding = args
        else:
            raise ValueError("Can't figure out the arguments")

        rospy.loginfo(
            'Waiting for image on topic %s, save to "%s"' % (self.topic, self.filename)
        )
        rospy.Subscriber(self.topic, Image, self.callback)

    def callback(self, msg):
        rospy.loginfo(
            'Received image on topic %s, encoding as %s, saving to "%s"'
            % (self.topic, self.encoding, self.filename)
        )

        # Why the hell do I need to do this?   Weird de-encoding issue in cv_bridge
        msg.data = bytearray(map(ord, msg.data))

        bridge = CvBridge()
        rospy.loginfo(bridge.encoding_to_dtype_with_channels(msg.encoding))
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding)
        cv2.imwrite(self.filename, cv_image)

        self.captured = True


if __name__ == "__main__":
    rospy.init_node("image_oneshot", anonymous=True)

    parser = argparse.ArgumentParser(
        prog="image_oneshot.py", description="What the program does"
    )

    parser.add_argument("--input-image", action="append", help="Input image topics")

    parser.add_argument("--camera-info", action="append", help="Input image topics")

    parser.add_argument("--output-image", action="append", help="Output image topics")

    parser.add_argument(
        "--timeout",
        default=5.0,
        type=float,
        help="Timeout while waiting for output topics in seconds (default 5.0)",
    )

    args = parser.parse_args()
    print(args)

    # Split out the camera info topics
    input_images = []
    camera_info = []
    output_images = []

    if args.input_image:
        input_images = [InputImagePub(i) for i in args.input_image]

    if args.camera_info:
        camera_info = [CameraInfoPub(i) for i in args.camera_info]

    if args.output_image:
        output_images = [OutputImageSub(i) for i in args.output_image]

    # Trigger all subscribers
    start_time = rospy.Time.now()

    # Wait for publishers to get registered (hacky)
    rospy.sleep(1)

    rospy.loginfo(
        "Sending %d image and %d camera info topics"
        % (len(input_images), len(camera_info))
    )
    for s in input_images:
        s.publish(start_time)

    for s in camera_info:
        s.publish(start_time)

    rospy.loginfo("Waiting for %d output images" % len(output_images))
    # Wait until all subscribers have fired (or timeout)
    done = False
    while not done:

        results = [output.captured for output in output_images]

        if False not in results:
            break

        dt = rospy.Time.now() - start_time
        if dt >= rospy.Duration(args.timeout):
            rospy.logwarn("Timeout")
            break

        rospy.sleep(0.5)
