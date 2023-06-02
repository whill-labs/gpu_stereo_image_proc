/*********************************************************************
 *  Copyright (c) 2023 University of Washington
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <memory>
#include <opencv2/calib3d/calib3d.hpp>

#include "gpu_stereo_image_proc/camera_info_conversions.h"
#include "gpu_stereo_image_proc/msg_conversions.h"
// #include
// "gpu_stereo_image_proc/visionworks/vx_bidirectional_stereo_matcher.h"
#include "gpu_stereo_image_proc/vpi/vpi_stereo_matcher.h"
#include "gpu_stereo_image_proc_common/DisparityBilateralFilterConfig.h"
#include "gpu_stereo_image_proc_common/DisparityWLSFilterConfig.h"
#include "gpu_stereo_image_proc_vpi/VPI_SGBMConfig.h"

namespace gpu_stereo_image_proc_vpi {

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class VPIDisparityNodelet : public nodelet::Nodelet {
  boost::shared_ptr<image_transport::ImageTransport> it_;

  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo>
      ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_disparity_, debug_lr_disparity_, debug_rl_disparity_;
  ros::Publisher debug_raw_disparity_, debug_disparity_mask_, pub_confidence_;
  ros::Publisher pub_depth_;

  ros::Publisher scaled_left_camera_info_, scaled_right_camera_info_;
  ros::Publisher scaled_left_rect_;

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef VPI_SGBMConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  typedef gpu_stereo_image_proc::DisparityBilateralFilterConfig
      BilateralFilterConfig;
  boost::shared_ptr<dynamic_reconfigure::Server<BilateralFilterConfig>>
      dyncfg_bilateral_filter_;

  typedef gpu_stereo_image_proc::DisparityWLSFilterConfig WLSFilterConfig;
  boost::shared_ptr<dynamic_reconfigure::Server<WLSFilterConfig>>
      dyncfg_wls_filter_;

  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;

  VPIStereoMatcherParams params_;
  std::shared_ptr<VPIStereoMatcher> stereo_matcher_;
  bool debug_topics_;

  virtual void onInit();

  void connectCb();

  void imageCb(const ImageConstPtr &l_image_msg,
               const CameraInfoConstPtr &l_info_msg,
               const ImageConstPtr &r_image_msg,
               const CameraInfoConstPtr &r_info_msg);

  void configCb(Config &config, uint32_t level);

  void bilateralConfigCb(BilateralFilterConfig &config, uint32_t level);
  void wlsConfigCb(WLSFilterConfig &config, uint32_t level);

  bool update_stereo_matcher();

 public:
  VPIDisparityNodelet();
};

VPIDisparityNodelet::VPIDisparityNodelet() {}

void VPIDisparityNodelet::onInit() {
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
        boost::bind(&VPIDisparityNodelet::imageCb, this, _1, _2, _3, _4));
  } else {
    exact_sync_.reset(new ExactSync(ExactPolicy(queue_size), sub_l_image_,
                                    sub_l_info_, sub_r_image_, sub_r_info_));
    exact_sync_->registerCallback(
        boost::bind(&VPIDisparityNodelet::imageCb, this, _1, _2, _3, _4));
  }

  // Set up dynamic reconfiguration
  ReconfigureServer::CallbackType f =
      boost::bind(&VPIDisparityNodelet::configCb, this, _1, _2);
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb =
      boost::bind(&VPIDisparityNodelet::connectCb, this);

  // Make sure we don't enter connectCb() between advertising and assigning to
  // pub_disparity_
  {
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_disparity_ =
        nh.advertise<DisparityImage>("disparity", 1, connect_cb, connect_cb);

    pub_depth_ = nh.advertise<Image>("depth", 1, connect_cb, connect_cb);

    pub_confidence_ =
        nh.advertise<Image>("confidence", 1, connect_cb, connect_cb);

    private_nh.param("debug", debug_topics_, false);
    if (debug_topics_) {
      ROS_INFO("Publishing debug topics");
      debug_lr_disparity_ =
          nh.advertise<DisparityImage>("debug/lr_disparity", 1);

      debug_rl_disparity_ =
          nh.advertise<DisparityImage>("debug/rl_disparity", 1);

      debug_raw_disparity_ = nh.advertise<DisparityImage>(
          "debug/raw_disparity", 1, connect_cb, connect_cb);

      debug_disparity_mask_ = nh.advertise<Image>("debug/confidence_mask", 1);
    }

    scaled_left_camera_info_ =
        nh.advertise<CameraInfo>("left/scaled_camera_info", 1);
    scaled_right_camera_info_ =
        nh.advertise<CameraInfo>("right/scaled_camera_info", 1);

    scaled_left_rect_ = nh.advertise<Image>("left/scaled_image_rect", 1);
  }
}

// Handles (un)subscribing when clients (un)subscribe
void VPIDisparityNodelet::connectCb() {
  boost::lock_guard<boost::mutex> lock(connect_mutex_);

  // If none of the relevant topics have subsubscribers, then we
  // can relax...
  if ((pub_disparity_.getNumSubscribers() == 0) &&
      (pub_confidence_.getNumSubscribers() == 0) &&
      (pub_depth_.getNumSubscribers() == 0)) {
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

void VPIDisparityNodelet::imageCb(const ImageConstPtr &l_image_msg,
                                  const CameraInfoConstPtr &l_info_msg,
                                  const ImageConstPtr &r_image_msg,
                                  const CameraInfoConstPtr &r_info_msg) {
  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);

  // Create cv::Mat views in the two input buffers
  const cv::Mat_<uint8_t> l_image =
      cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)
          ->image;
  const cv::Mat_<uint8_t> r_image =
      cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)
          ->image;

  params_.set_image_size(cv::Size(l_image.cols, l_image.rows));

  if (stereo_matcher_) {
    const cv::Size image_size = stereo_matcher_->params().image_size();
    if (image_size.width != l_image.cols || image_size.height != l_image.rows) {
      update_stereo_matcher();
    }
  } else {
    update_stereo_matcher();
  }

  // If you **still** don't have a stereo matcher, give up...
  if (!stereo_matcher_) return;

  // // Pull in some parameters as constants
  const int min_disparity = 0;
  const int max_disparity = stereo_matcher_->params().max_disparity;
  const int downsample = stereo_matcher_->params().downsample();
  const int border = stereo_matcher_->params().window_size / 2;

  const auto scaled_camera_info_l(scaleCameraInfo(l_info_msg, downsample));
  const auto scaled_camera_info_r(scaleCameraInfo(r_info_msg, downsample));

  image_geometry::StereoCameraModel scaled_model;
  scaled_model.fromCameraInfo(scaled_camera_info_l, scaled_camera_info_r);

  // Block matcher produces 16-bit signed (fixed point) disparity image
  stereo_matcher_->compute(l_image, r_image);
  cv::Mat_<int16_t> disparityS16 = stereo_matcher_->disparity();

  // if (debug_topics_) {
  //   DisparityImageGenerator raw_dg(l_image_msg, disparityS16, scaled_model,
  //                                  min_disparity, max_disparity, border);
  //   debug_raw_disparity_.publish(raw_dg.getDisparity());
  // }

  // if (std::shared_ptr<VXBidirectionalStereoMatcher> bm =
  //         std::dynamic_pointer_cast<VXBidirectionalStereoMatcher>(
  //             stereo_matcher_)) {
  //   // Publish confidence
  //   cv::Mat confidence = bm->confidenceMat();
  //   cv_bridge::CvImage confidence_bridge(l_image_msg->header, "32FC1",
  //                                        confidence);
  //   pub_confidence_.publish(confidence_bridge.toImageMsg());

  //   if (confidence_threshold_ > 0) {
  //     // Since we use the mask to **discard** disparities with low
  //     confidence,
  //     // use CMP_LT to **set** pixels in the mask which have a confidence
  //     // below the threshold.
  //     cv::Mat confidence_mask(confidence.size(), CV_8UC1, cv::Scalar(0));
  //     cv::compare(confidence, confidence_threshold_, confidence_mask,
  //                 cv::CMP_LT);

  //     if (debug_topics_) {
  //       cv_bridge::CvImage disparity_mask_bridge(l_image_msg->header,
  //       "mono8",
  //                                                confidence_mask);
  //       debug_disparity_mask_.publish(disparity_mask_bridge.toImageMsg());
  //     }

  //     // Erase disparity values with low confidence
  //     const int masked_disparity_value = min_disparity;
  //     disparityS16.setTo(masked_disparity_value, confidence_mask);
  //   }
  // }

  cv::Mat confidence = stereo_matcher_->confidence();
  cv_bridge::CvImage confidence_bridge(l_image_msg->header, "16SC1",
                                       confidence);
  pub_confidence_.publish(confidence_bridge.toImageMsg());

  DisparityImageGenerator dg(l_image_msg, disparityS16, scaled_model,
                             min_disparity, max_disparity, border);

  pub_disparity_.publish(dg.getDisparity());
  pub_depth_.publish(dg.getDepth());

  scaled_left_camera_info_.publish(scaled_camera_info_l);
  scaled_right_camera_info_.publish(scaled_camera_info_r);

  // cv_bridge::CvImage left_rect_msg_bridge(l_image_msg->header, "mono8",
  //                                         stereo_matcher_->scaledLeftRect());
  // scaled_left_rect_.publish(left_rect_msg_bridge.toImageMsg());

  // if (debug_topics_) {
  //   // This results in an copy of the mat, so only do it if necessary..
  //   cv::Mat scaledDisparity = stereo_matcher_->unfilteredDisparityMat();
  //   if (!scaledDisparity.empty()) {
  //     DisparityImageGenerator lr_disp_dg(l_image_msg, scaledDisparity,
  //                                        scaled_model, min_disparity,
  //                                        max_disparity, border);
  //     DisparityImagePtr lr_disp_msg = lr_disp_dg.getDisparity();
  //     debug_lr_disparity_.publish(lr_disp_msg);
  //   }

  //   if (std::shared_ptr<VXBidirectionalStereoMatcher> bm =
  //           std::dynamic_pointer_cast<VXBidirectionalStereoMatcher>(
  //               stereo_matcher_)) {
  //     // This results in an copy of the mat, so only do it if necessary..
  //     cv::Mat rlScaledDisparity = bm->RLDisparityMat();
  //     if (!rlScaledDisparity.empty()) {
  //       DisparityImageGenerator rl_disp_dg(l_image_msg, rlScaledDisparity,
  //                                          scaled_model, min_disparity,
  //                                          max_disparity, border);
  //       DisparityImagePtr rl_disp_msg = rl_disp_dg.getDisparity();
  //       debug_rl_disparity_.publish(rl_disp_msg);
  //     }
  //   }
  // }
}

void VPIDisparityNodelet::configCb(Config &config, uint32_t level) {
  // Settings for the nodelet itself
  // confidence_threshold_ = config.confidence_threshold;

  // config.correlation_window_size |= 0x1;  // must be odd
  params_.window_size = config.correlation_window_size;

  // config.max_disparity = (config.max_disparity / 4) * 4;
  params_.max_disparity = config.max_disparity;

  params_.downsample_log2 = config.downsample;
  params_.quality = config.quality;
  params_.confidence_threshold = config.confidence_threshold;

  // if (config.disparity_filter == VXSGBM_BilateralFilter) {
  //   ROS_INFO("Enabling bilateral filtering");
  //   params_.filtering = VXStereoMatcherParams::Filtering_Bilateral;
  //   // } else if (config.disparity_filter == VXSGBM_WLSFilter_LeftOnly) {
  //   //   ROS_INFO("Enabling Left-only WLS filtering");
  //   //   params_.filtering = VXStereoMatcherParams::Filtering_WLS_LeftOnly;
  // } else if (config.disparity_filter == VXSGBM_WLSFilter_LeftRight) {
  //   ROS_INFO("Enabling Left-Right WLS filtering");
  //   params_.filtering = VXStereoMatcherParams::Filtering_WLS_LeftRight;
  // } else {
  //   ROS_INFO("Disabling filtering");
  //   params_.filtering = VXStereoMatcherParams::Filtering_None;
  // }

  update_stereo_matcher();
}

void VPIDisparityNodelet::bilateralConfigCb(BilateralFilterConfig &config,
                                            uint32_t level) {
  params_.bilateral_filter_params.sigma_range = config.sigma_range;
  params_.bilateral_filter_params.radius = config.radius;
  params_.bilateral_filter_params.num_iters = config.num_iters;
  params_.bilateral_filter_params.max_disc_threshold =
      config.max_disc_threshold;
  params_.bilateral_filter_params.edge_threshold = config.edge_threshold;

  update_stereo_matcher();
}

void VPIDisparityNodelet::wlsConfigCb(WLSFilterConfig &config, uint32_t level) {
  params_.wls_filter_params.lambda = config.lambda;
  params_.wls_filter_params.lrc_threshold = config.lrc_threshold;

  update_stereo_matcher();
}

bool VPIDisparityNodelet::update_stereo_matcher() {
  // \todo For safety, should mutex this and imageCb
  ROS_WARN("Updating stereo_matcher");

  if (!params_.valid()) {
    ROS_WARN("Stereo matcher params are not valid...");
    return false;
  }

  params_.dump();
  ROS_WARN("Creating new stereo_matcher");
  // if (params_.filtering == VXStereoMatcherParams::Filtering_WLS_LeftRight) {
  //   ROS_INFO("Creating VXBidirectionalStereoMatcher");
  //   stereo_matcher_.reset(new VXBidirectionalStereoMatcher(params_));
  // } else {
  stereo_matcher_.reset(new VPIStereoMatcher(params_));
  //}
  return true;
}

}  // namespace gpu_stereo_image_proc_vpi

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gpu_stereo_image_proc_vpi::VPIDisparityNodelet,
                       nodelet::Nodelet)