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

#include "gpu_stereo_image_proc/nodelet_base.h"
#include "gpu_stereo_image_proc/visionworks/vx_bidirectional_stereo_matcher.h"
#include "gpu_stereo_image_proc/visionworks/vx_stereo_matcher.h"
#include "gpu_stereo_image_proc_common/DisparityBilateralFilterConfig.h"
#include "gpu_stereo_image_proc_common/DisparityWLSFilterConfig.h"
#include "gpu_stereo_image_proc_visionworks/VXSGBMConfig.h"

namespace gpu_stereo_image_proc_visionworks {
using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class VXDisparityNodelet : public gpu_stereo_image_proc::DisparityNodeletBase {
  // Publications
  ros::Publisher pub_disparity_, pub_depth_, pub_confidence_;

  // Debug-only topics
  ros::Publisher debug_lr_disparity_, debug_rl_disparity_;
  ros::Publisher debug_raw_disparity_, debug_disparity_mask_;

  ros::Publisher scaled_left_camera_info_, scaled_right_camera_info_;
  ros::Publisher scaled_left_rect_;

  // Dynamic reconfigure
  typedef VXSGBMConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  typedef gpu_stereo_image_proc::DisparityBilateralFilterConfig
      BilateralFilterConfig;
  boost::shared_ptr<dynamic_reconfigure::Server<BilateralFilterConfig>>
      dyncfg_bilateral_filter_;

  typedef gpu_stereo_image_proc::DisparityWLSFilterConfig WLSFilterConfig;
  boost::shared_ptr<dynamic_reconfigure::Server<WLSFilterConfig>>
      dyncfg_wls_filter_;

  VXStereoMatcherParams params_;
  std::shared_ptr<VXStereoMatcher> stereo_matcher_;
  bool debug_topics_;

  unsigned int confidence_threshold_;

  void onInit() override;

  bool hasSubscribers() const override {
    return (pub_disparity_.getNumSubscribers() > 0) ||
           (pub_confidence_.getNumSubscribers() > 0) ||
           (pub_depth_.getNumSubscribers() > 0);
  }

  int downsample() const override { return params_.downsample(); }

  void imageCallback(const ImageConstPtr &l_image_msg,
                     const CameraInfoConstPtr &l_info_msg,
                     const ImageConstPtr &r_image_msg,
                     const CameraInfoConstPtr &r_info_msg) override;

  void configCb(Config &config, uint32_t level);
  void bilateralConfigCb(BilateralFilterConfig &config, uint32_t level);
  void wlsConfigCb(WLSFilterConfig &config, uint32_t level);

  bool update_stereo_matcher();

 public:
  VXDisparityNodelet() : confidence_threshold_(0) { ; }
};

void VXDisparityNodelet::onInit() {
  DisparityNodeletBase::onInit();

  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();

  // Set up dynamic reconfiguration
  ReconfigureServer::CallbackType f =
      boost::bind(&VXDisparityNodelet::configCb, this, _1, _2);
  reconfigure_server_.reset(new ReconfigureServer(private_nh));
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb =
      boost::bind(&DisparityNodeletBase::connectCb, this);

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

void VXDisparityNodelet::imageCallback(const ImageConstPtr &l_image_msg,
                                       const CameraInfoConstPtr &l_info_msg,
                                       const ImageConstPtr &r_image_msg,
                                       const CameraInfoConstPtr &r_info_msg) {
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

  // Block matcher produces 16-bit signed (fixed point) disparity image
  stereo_matcher_->compute(l_image, r_image);
  cv::Mat_<int16_t> disparityS16 = stereo_matcher_->disparity();

  DisparityImageGenerator dg(scaled_model_,
                             stereo_matcher_->params().min_disparity,
                             stereo_matcher_->params().max_disparity,
                             stereo_matcher_->params().sad_win_size / 2);

  if (debug_topics_) {
    debug_raw_disparity_.publish(
        dg.generate(l_image_msg, disparityS16).getDisparity());
  }

  if (std::shared_ptr<VXBidirectionalStereoMatcher> bm =
          std::dynamic_pointer_cast<VXBidirectionalStereoMatcher>(
              stereo_matcher_)) {
    // Publish confidence
    cv::Mat confidence = bm->confidenceMat();
    cv_bridge::CvImage confidence_bridge(l_image_msg->header, "32FC1",
                                         confidence);
    pub_confidence_.publish(confidence_bridge.toImageMsg());

    if (confidence_threshold_ > 0) {
      // Since we use the mask to **discard** disparities with low confidence,
      // use CMP_LT to **set** pixels in the mask which have a confidence
      // below the threshold.
      cv::Mat confidence_mask(confidence.size(), CV_8UC1, cv::Scalar(0));
      cv::compare(confidence, confidence_threshold_, confidence_mask,
                  cv::CMP_LT);

      if (debug_topics_) {
        cv_bridge::CvImage disparity_mask_bridge(l_image_msg->header, "mono8",
                                                 confidence_mask);
        debug_disparity_mask_.publish(disparity_mask_bridge.toImageMsg());
      }

      // Erase disparity values with low confidence
      const int masked_disparity_value =
          stereo_matcher_->params().min_disparity;
      disparityS16.setTo(masked_disparity_value, confidence_mask);
    }
  }

  auto disparity_image(dg.generate(l_image_msg, disparityS16));

  pub_disparity_.publish(disparity_image.getDisparity());
  pub_depth_.publish(disparity_image.getDepth());

  scaled_left_camera_info_.publish(scaled_model_.left().cameraInfo());
  scaled_right_camera_info_.publish(scaled_model_.right().cameraInfo());

  cv_bridge::CvImage left_rect_msg_bridge(l_image_msg->header, "mono8",
                                          stereo_matcher_->scaledLeftRect());
  scaled_left_rect_.publish(left_rect_msg_bridge.toImageMsg());

  if (debug_topics_) {
    // This results in an copy of the mat, so only do it if necessary..
    cv::Mat scaledDisparity = stereo_matcher_->unfilteredDisparityMat();
    if (!scaledDisparity.empty()) {
      debug_lr_disparity_.publish(
          dg.generate(l_image_msg, scaledDisparity).getDisparity());
    }

    if (std::shared_ptr<VXBidirectionalStereoMatcher> bm =
            std::dynamic_pointer_cast<VXBidirectionalStereoMatcher>(
                stereo_matcher_)) {
      // This results in an copy of the mat, so only do it if necessary..
      cv::Mat rlScaledDisparity = bm->RLDisparityMat();
      if (!rlScaledDisparity.empty()) {
        debug_rl_disparity_.publish(
            dg.generate(l_image_msg, rlScaledDisparity).getDisparity());
      }
    }
  }
}

void VXDisparityNodelet::configCb(Config &config, uint32_t level) {
  // Settings for the nodelet itself
  confidence_threshold_ = config.confidence_threshold;

  // Tweak all settings to be valid
  config.correlation_window_size |= 0x1;  // must be odd
  config.max_disparity = (config.max_disparity / 4) * 4;

  int scanline_mask = 0;
  if (config.path_type == VXSGBM_SCANLINE_ALL) {
    scanline_mask = NVX_SCANLINE_ALL;
  } else if (config.path_type == VXSGBM_SCANLINE_CROSS) {
    scanline_mask = NVX_SCANLINE_CROSS;
  } else {
    if (config.SCANLINE_LEFT_RIGHT) scanline_mask |= NVX_SCANLINE_LEFT_RIGHT;
    if (config.SCANLINE_TOP_LEFT_BOTTOM_RIGHT)
      scanline_mask |= NVX_SCANLINE_TOP_LEFT_BOTTOM_RIGHT;
    if (config.SCANLINE_TOP_BOTTOM) scanline_mask |= NVX_SCANLINE_TOP_BOTTOM;
    if (config.SCANLINE_TOP_RIGHT_BOTTOM_LEFT)
      scanline_mask |= NVX_SCANLINE_TOP_RIGHT_BOTTOM_LEFT;
    if (config.SCANLINE_RIGHT_LEFT) scanline_mask |= NVX_SCANLINE_RIGHT_LEFT;
    if (config.SCANLINE_BOTTOM_RIGHT_TOP_LEFT)
      scanline_mask |= NVX_SCANLINE_BOTTOM_RIGHT_TOP_LEFT;
    if (config.SCANLINE_BOTTOM_TOP) scanline_mask |= NVX_SCANLINE_BOTTOM_TOP;
    if (config.SCANLINE_BOTTOM_LEFT_TOP_RIGHT)
      scanline_mask |= NVX_SCANLINE_BOTTOM_LEFT_TOP_RIGHT;
  }

  int flags = 0;
  if (config.FILTER_TOP_AREA) flags |= NVX_SGM_FILTER_TOP_AREA;
  if (config.PYRAMIDAL_STEREO) flags |= NVX_SGM_PYRAMIDAL_STEREO;

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
  params_.downsample_log2 = config.downsample;

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

}  // namespace gpu_stereo_image_proc_visionworks

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gpu_stereo_image_proc_visionworks::VXDisparityNodelet,
                       nodelet::Nodelet)
