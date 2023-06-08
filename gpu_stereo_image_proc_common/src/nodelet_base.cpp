
#include "gpu_stereo_image_proc/nodelet_base.h"

namespace gpu_stereo_image_proc {

DisparityNodeletBase::DisparityNodeletBase() {}

void DisparityNodeletBase::onInit() {
  // Use the single-threaded model.   There are really only two callbacks:
  // config and new images.  Don't want to have to deconflict those
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();

  it_.reset(new image_transport::ImageTransport(nh));

  // Synchronize inputs. Topic subscriptions happen on demand in the connection
  // callback. Optionally do approximate synchronization.
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  bool approx;
  private_nh.param("approximate_sync", approx, false);
  if (approx) {
    approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size),
                                                sub_l_image_, sub_l_info_,
                                                sub_r_image_, sub_r_info_));
    approximate_sync_->registerCallback(
        boost::bind(&DisparityNodeletBase::imageCb, this, _1, _2, _3, _4));
  } else {
    exact_sync_.reset(new ExactSync(ExactPolicy(queue_size), sub_l_image_,
                                    sub_l_info_, sub_r_image_, sub_r_info_));
    exact_sync_->registerCallback(
        boost::bind(&DisparityNodeletBase::imageCb, this, _1, _2, _3, _4));
  }

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb =
      boost::bind(&DisparityNodeletBase::connectCb, this);
}

// Handles (un)subscribing when clients (un)subscribe
void DisparityNodeletBase::connectCb() {
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  // If none of the relevant topics have subsubscribers, then we
  // can relax...
  if (!hasSubscribers()) {
    sub_l_image_.unsubscribe();
    sub_l_info_.unsubscribe();
    sub_r_image_.unsubscribe();
    sub_r_info_.unsubscribe();
  } else if (!sub_l_image_.getSubscriber()) {
    ros::NodeHandle &nh = getNodeHandle();
    ROS_DEBUG("Client connecting, subscribing to images...");

    // Queue size 1 should be OK; the one that matters is the synchronizer queue
    // size.
    /// @todo Allow remapping left, right?
    image_transport::TransportHints hints("raw", ros::TransportHints(),
                                          getPrivateNodeHandle());
    sub_l_image_.subscribe(*it_, "left/image_rect", 1, hints);
    sub_l_info_.subscribe(nh, "left/camera_info", 1);
    sub_r_image_.subscribe(*it_, "right/image_rect", 1, hints);
    sub_r_info_.subscribe(nh, "right/camera_info", 1);
  }
}

void DisparityNodeletBase::imageCb(const ImageConstPtr &l_image_msg,
                                   const CameraInfoConstPtr &l_info_msg,
                                   const ImageConstPtr &r_image_msg,
                                   const CameraInfoConstPtr &r_info_msg) {
  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);

  const auto scaled_camera_info_l(scaleCameraInfo(l_info_msg, downsample()));
  const auto scaled_camera_info_r(scaleCameraInfo(r_info_msg, downsample()));

  scaled_model_.fromCameraInfo(scaled_camera_info_l, scaled_camera_info_r);

  imageCallback(l_image_msg, l_info_msg, r_image_msg, r_info_msg);
}

}  // namespace gpu_stereo_image_proc
