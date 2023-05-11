#include <gpu_stereo_image_proc/camera_info_conversions.h>
#include <ros/ros.h>

sensor_msgs::CameraInfoPtr scaleCameraInfo(
    const sensor_msgs::CameraInfoConstPtr &cam, float downsample) {
  sensor_msgs::CameraInfoPtr out =
      boost::make_shared<sensor_msgs::CameraInfo>();

  out->header = cam->header;

  out->width = cam->width / downsample;
  out->height = cam->height / downsample;

  // Distortion parameters aaren't meaningful after scaling
  //  out->distortion_model
  //  out->D

  out->K[0] = cam->K[0] / downsample;
  out->K[2] = cam->K[2] / downsample;
  out->K[4] = cam->K[4] / downsample;
  out->K[5] = cam->K[5] / downsample;
  out->K[8] = 1;

  out->R = cam->R;

  out->P[0] = cam->P[0] / downsample;
  out->P[2] = cam->P[2] / downsample;
  out->P[3] = cam->P[3] / downsample;

  out->P[5] = cam->P[5] / downsample;
  out->P[6] = cam->P[6] / downsample;
  out->P[7] = cam->P[7] / downsample;

  out->P[10] = 1;

  return out;
}
