/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, WHILL, Inc.
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

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>z
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>

#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Image.h>

#include "gpu_stereo_image_proc_common/DisparityBilateralFilterConfig.h"
#include "gpu_stereo_image_proc_common/DisparityWLSFilterConfig.h"
#include "gpu_stereo_image_proc_visionworks/VXSGBMConfig.h"
#include <dynamic_reconfigure/server.h>

#include "gpu_stereo_image_proc/camera_info_conversions.h"
#include "gpu_stereo_image_proc/msg_conversions.h"
#include "gpu_stereo_image_proc/visionworks/vx_bidirectional_stereo_matcher.h"
#include "gpu_stereo_image_proc/visionworks/vx_stereo_matcher.h"

namespace gpu_stereo_image_proc {
using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class VXDisparityNodelet : public nodelet::Nodelet {
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
  typedef gpu_stereo_image_proc::VXSGBMConfig Config;
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

  VXStereoMatcherParams params_;
  std::shared_ptr<VXStereoMatcherBase> stereo_matcher_;
  bool debug_topics_;

  unsigned int confidence_threshold_;

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
    VXDisparityNodelet() : confidence_threshold_(0) { ; }

};

void VXDisparityNodelet::onInit() {
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
        boost::bind(&VXDisparityNodelet::imageCb, this, _1, _2, _3, _4));
  } else {
    exact_sync_.reset(new ExactSync(ExactPolicy(queue_size), sub_l_image_,
                                    sub_l_info_, sub_r_image_, sub_r_info_));
    exact_sync_->registerCallback(
        boost::bind(&VXDisparityNodelet::imageCb, this, _1, _2, _3, _4));
  }

  // Set up dynamic reconfiguration
  ReconfigureServer::CallbackType f =
      boost::bind(&VXDisparityNodelet::configCb, this, _1, _2);
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb =
      boost::bind(&VXDisparityNodelet::connectCb, this);

  // Make sure we don't enter connectCb() between advertising and assigning to
  // pub_disparity_
  {
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_disparity_ =
        nh.advertise<DisparityImage>("disparity", 1, connect_cb, connect_cb);

    pub_depth_ =
        nh.advertise<Image>("depth", 1, connect_cb, connect_cb);

    pub_confidence_ =
        nh.advertise<Image>("confidence", 1, connect_cb, connect_cb);

    private_nh.param("debug", debug_topics_, false);
    if (debug_topics_) {
      ROS_INFO("Publishing debug topics");
      debug_lr_disparity_ = nh.advertise<DisparityImage>("debug/lr_disparity", 1);

      debug_rl_disparity_ = nh.advertise<DisparityImage>("debug/rl_disparity", 1);

    debug_raw_disparity_ =
        nh.advertise<DisparityImage>("debug/raw_disparity", 1, connect_cb, connect_cb);

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
void VXDisparityNodelet::connectCb() {
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if ((pub_disparity_.getNumSubscribers() == 0) &&
      (pub_confidence_.getNumSubscribers() == 0)) {
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

void VXDisparityNodelet::imageCb(const ImageConstPtr &l_image_msg,
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
  if (!stereo_matcher_)
    return;

  // Pull in some parameters as constants
  const int min_disparity = stereo_matcher_->params().min_disparity;
  const int max_disparity = stereo_matcher_->params().max_disparity;
  const int downsample = stereo_matcher_->params().downsample;
  const int border = stereo_matcher_->params().sad_win_size / 2;

  const auto scaled_camera_info_l(scaleCameraInfo(l_info_msg, downsample));
  const auto scaled_camera_info_r(scaleCameraInfo(r_info_msg, downsample));
  image_geometry::StereoCameraModel scaled_model;
  scaled_model.fromCameraInfo(scaled_camera_info_l, scaled_camera_info_r);

  // Block matcher produces 16-bit signed (fixed point) disparity image
  cv::Mat_<int16_t> disparityS16;
  stereo_matcher_->compute(l_image, r_image, disparityS16);
  DisparityImageGenerator dg(l_image_msg, disparityS16, scaled_model,
                                min_disparity, max_disparity, border);
  DisparityImagePtr disp_msg = dg.getDisparity();
  DisparityImageGenerator dg_for_depth = dg;

  if (debug_topics_) debug_raw_disparity_.publish(disp_msg);

  if (std::shared_ptr<VXBidirectionalStereoMatcher> bm =
          std::dynamic_pointer_cast<VXBidirectionalStereoMatcher>(
              stereo_matcher_)) {

    // Publish confidence
    cv::Mat confidence = bm->confidenceMat();

    cv_bridge::CvImage confidence_bridge(l_image_msg->header, "32FC1",
                                         confidence);
    pub_confidence_.publish(confidence_bridge.toImageMsg());

    if (confidence_threshold_ > 0) {

      // Convert confidence to 8UC1
      cv::Mat confidence_8uc1(confidence.size(), CV_8UC1);

      //WLS filter produces float confidences between 0 and 255, so we don't need to scale
      confidence.convertTo(confidence_8uc1, CV_8UC1);
      // Yes, filter on confidence
      cv::Mat confidence_mask(confidence.size(), CV_8UC1, cv::Scalar(0));
      cv::threshold(confidence_8uc1, confidence_mask, confidence_threshold_, 255,
                     cv::THRESH_BINARY);

      if( debug_topics_) {
        cv_bridge::CvImage disparity_mask_bridge(l_image_msg->header, "mono8", confidence_mask);
        debug_disparity_mask_.publish(disparity_mask_bridge.toImageMsg());
      }

      cv::Mat masked_disparityS16;
      disparityS16.copyTo(masked_disparityS16, confidence_mask);

      
      DisparityImageGenerator masked_dg(l_image_msg, disparityS16, scaled_model,
                                min_disparity, max_disparity, border);
      DisparityImagePtr masked_disp_msg = masked_dg.getDisparity();
      pub_disparity_.publish(masked_disp_msg);
      dg_for_depth = masked_dg;

    } else {
      // No, don't filter on confidence
      pub_disparity_.publish(disp_msg);
      dg_for_depth = dg;
    }
  } else {
    pub_disparity_.publish(disp_msg);
    dg_for_depth = dg;
  }

  // Publish a depth image of type sensor_msgs/Image
  ImagePtr depth_msg = disparityImageToDepthImage(disp_msg);
  pub_depth_.publish(depth_msg);

  scaled_left_camera_info_.publish(scaled_camera_info_l);
  scaled_right_camera_info_.publish(scaled_camera_info_r);

  // Mildly inefficient but good for now...
  // I need a scaled rectified image for point cloud coloring
  cv::Mat scaledLeftRect;
  cv::resize(l_image, scaledLeftRect, cv::Size(), 1.0 / downsample,
             1.0 / downsample);
  cv_bridge::CvImage left_rect_msg_bridge(l_image_msg->header, "mono8",
                                          scaledLeftRect);
  scaled_left_rect_.publish(left_rect_msg_bridge.toImageMsg());

  if (debug_topics_) {
    // This is a copy, so only do it if necessary..
    cv::Mat scaledDisparity = stereo_matcher_->unfilteredDisparityMat();
    if (!scaledDisparity.empty()) {
      DisparityImageGenerator rl_disp_dg(l_image_msg, disparityS16, scaled_model,
                                min_disparity, max_disparity, border, downsample);
        DisparityImagePtr rl_disp_msg = rl_disp_dg.getDisparity();
      debug_lr_disparity_.publish(lr_disp_msg);
    }

    if (std::shared_ptr<VXBidirectionalStereoMatcher> bm =
            std::dynamic_pointer_cast<VXBidirectionalStereoMatcher>(
                stereo_matcher_)) {

      // This is a copy, so only do it if necessary..
      cv::Mat rlScaledDisparity = bm->RLDisparityMat();
      if (!rlScaledDisparity.empty()) {
        DisparityImageGenerator rl_disp_dg(l_image_msg, disparityS16, scaled_model,
                                min_disparity, max_disparity, border, downsample);
        DisparityImagePtr rl_disp_msg = rl_disp_dg.getDisparity();
        debug_rl_disparity_.publish(rl_disp_msg);
      }
    }
  }
} // namespace gpu_stereo_image_proc

void VXDisparityNodelet::configCb(Config &config, uint32_t level) {
  // Settings for the nodelet itself
  confidence_threshold_ = config.confidence_threshold;

  // Tweak all settings to be valid
  config.correlation_window_size |= 0x1; // must be odd
  config.max_disparity = (config.max_disparity / 4) * 4;
  config.downsample =
      static_cast<int>(pow(2, static_cast<int>(log2(config.downsample))));

  int scanline_mask = 0;
  if (config.path_type == VXSGBM_SCANLINE_ALL) {
    scanline_mask = NVX_SCANLINE_ALL;
  } else if (config.path_type == VXSGBM_SCANLINE_CROSS) {
    scanline_mask = NVX_SCANLINE_CROSS;
  } else {
    if (config.SCANLINE_LEFT_RIGHT)
      scanline_mask |= NVX_SCANLINE_LEFT_RIGHT;
    if (config.SCANLINE_TOP_LEFT_BOTTOM_RIGHT)
      scanline_mask |= NVX_SCANLINE_TOP_LEFT_BOTTOM_RIGHT;
    if (config.SCANLINE_TOP_BOTTOM)
      scanline_mask |= NVX_SCANLINE_TOP_BOTTOM;
    if (config.SCANLINE_TOP_RIGHT_BOTTOM_LEFT)
      scanline_mask |= NVX_SCANLINE_TOP_RIGHT_BOTTOM_LEFT;
    if (config.SCANLINE_RIGHT_LEFT)
      scanline_mask |= NVX_SCANLINE_RIGHT_LEFT;
    if (config.SCANLINE_BOTTOM_RIGHT_TOP_LEFT)
      scanline_mask |= NVX_SCANLINE_BOTTOM_RIGHT_TOP_LEFT;
    if (config.SCANLINE_BOTTOM_TOP)
      scanline_mask |= NVX_SCANLINE_BOTTOM_TOP;
    if (config.SCANLINE_BOTTOM_LEFT_TOP_RIGHT)
      scanline_mask |= NVX_SCANLINE_BOTTOM_LEFT_TOP_RIGHT;
  }

  int flags = 0;
  if (config.FILTER_TOP_AREA)
    flags |= NVX_SGM_FILTER_TOP_AREA;
  if (config.PYRAMIDAL_STEREO)
    flags |= NVX_SGM_PYRAMIDAL_STEREO;

  if (config.disparity_filter == VXSGBM_BilateralFilter) {
    ROS_INFO("Enabling bilateral filtering");
    params_.filtering = VXStereoMatcherParams::Filtering_Bilateral;
    // } else if (config.disparity_filter == VXSGBM_WLSFilter_LeftOnly) {
    //   ROS_INFO("Enabling Left-only WLS filtering");
    //   params_.filtering = VXStereoMatcherParams::Filtering_WLS_LeftOnly;
  } else if (config.disparity_filter == VXSGBM_WLSFilter_LeftRight) {
    ROS_INFO("Enabling Left-Right WLS filtering");
    params_.filtering = VXStereoMatcherParams::Filtering_WLS_LeftRight;
  } else {
    ROS_INFO("Disabling filtering");
    params_.filtering = VXStereoMatcherParams::Filtering_None;
  }

  // check stereo method
  // Note: With single-threaded NodeHandle, configCb and imageCb can't be called
  // concurrently, so this is thread-safe.
  params_.sad_win_size = config.correlation_window_size;
  params_.min_disparity = config.min_disparity;
  params_.max_disparity = config.max_disparity;
  params_.uniqueness_ratio = config.uniqueness_ratio;
  params_.P1 = config.P1;
  params_.P2 = config.P2;
  params_.max_diff = config.disp12MaxDiff;
  params_.clip = config.bt_clip_value;
  params_.ct_win_size = config.ct_win_size;
  params_.hc_win_size = config.hc_win_size;
  params_.flags = flags;
  params_.scanline_mask = scanline_mask;
  params_.downsample = config.downsample;

  update_stereo_matcher();
}

void VXDisparityNodelet::bilateralConfigCb(BilateralFilterConfig &config,
                                           uint32_t level) {
  params_.bilateral_filter_params.sigma_range = config.sigma_range;
  params_.bilateral_filter_params.radius = config.radius;
  params_.bilateral_filter_params.num_iters = config.num_iters;
  params_.bilateral_filter_params.max_disc_threshold =
      config.max_disc_threshold;
  params_.bilateral_filter_params.edge_threshold = config.edge_threshold;

  update_stereo_matcher();
}

void VXDisparityNodelet::wlsConfigCb(WLSFilterConfig &config, uint32_t level) {
  params_.wls_filter_params.lambda = config.lambda;
  params_.wls_filter_params.lrc_threshold = config.lrc_threshold;

  update_stereo_matcher();
}

bool VXDisparityNodelet::update_stereo_matcher() {
  // \todo For safety, should mutex this and imageCb
  ROS_WARN("Updating stereo_matcher");

  if (!params_.valid()) {
    ROS_WARN("Stereo matcher params are not valid...");
    return false;
  }

  params_.dump();
  ROS_WARN("Creating new stereo_matcher");
  if (params_.filtering == VXStereoMatcherParams::Filtering_WLS_LeftRight) {
    ROS_INFO("Creating VXBidirectionalStereoMatcher");
    stereo_matcher_.reset(new VXBidirectionalStereoMatcher(params_));
  } else {
    stereo_matcher_.reset(new VXStereoMatcher(params_));
  }
  return true;
}

} // namespace gpu_stereo_image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gpu_stereo_image_proc::VXDisparityNodelet,
                       nodelet::Nodelet)
