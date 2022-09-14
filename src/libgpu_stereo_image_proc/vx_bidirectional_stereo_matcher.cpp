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
#include <iostream>
#include <ros/ros.h>

#include "gpu_stereo_image_proc/visionworks/vx_bidirectional_stereo_matcher.h"
#include <NVX/nvx_opencv_interop.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

VXBidirectionalStereoMatcher::VXBidirectionalStereoMatcher()
    : VXStereoMatcherBase() {}

VXBidirectionalStereoMatcher::VXBidirectionalStereoMatcher(
    const VXStereoMatcherParams &params)
    : VXStereoMatcherBase(params) {
  vx_status status;

  if (params.shrink_scale > 1) {
    left_scaled_ = vxCreateImage(
        context_, params.image_size.width / params.shrink_scale,
        params.image_size.height / params.shrink_scale, VX_DF_IMAGE_U8);
    VX_CHECK_STATUS(vxGetStatus((vx_reference)left_scaled_));

    right_scaled_ = vxCreateImage(
        context_, params.image_size.width / params.shrink_scale,
        params.image_size.height / params.shrink_scale, VX_DF_IMAGE_U8);
    VX_CHECK_STATUS(vxGetStatus((vx_reference)right_scaled_));

    disparity_scaled_ = vxCreateImage(
        context_, params.image_size.width / params.shrink_scale,
        params.image_size.height / params.shrink_scale, VX_DF_IMAGE_S16);
    VX_CHECK_STATUS(vxGetStatus((vx_reference)disparity_scaled_));

    flipped_left_ = vxCreateImage(
        context_, params.image_size.width / params.shrink_scale,
        params.image_size.height / params.shrink_scale, VX_DF_IMAGE_U8);
    VX_CHECK_STATUS(vxGetStatus((vx_reference)flipped_left_));

    flipped_right_ = vxCreateImage(
        context_, params.image_size.width / params.shrink_scale,
        params.image_size.height / params.shrink_scale, VX_DF_IMAGE_U8);
    VX_CHECK_STATUS(vxGetStatus((vx_reference)flipped_right_));

    flipped_rl_disparity_scaled_ = vxCreateImage(
        context_, params.image_size.width / params.shrink_scale,
        params.image_size.height / params.shrink_scale, VX_DF_IMAGE_S16);
    VX_CHECK_STATUS(vxGetStatus((vx_reference)flipped_rl_disparity_scaled_));

    // rl_disparity_scaled_ = vxCreateImage(
    //     context_, params.image_size.width / params.shrink_scale,
    //     params.image_size.height / params.shrink_scale, VX_DF_IMAGE_S16);
    // VX_CHECK_STATUS(vxGetStatus((vx_reference)rl_disparity_scaled_));

    // negated_rl_disparity_scaled_ = vxCreateImage(
    //     context_, params.image_size.width / params.shrink_scale,
    //     params.image_size.height / params.shrink_scale, VX_DF_IMAGE_S16);
    // VX_CHECK_STATUS(vxGetStatus((vx_reference)negated_rl_disparity_scaled_));

    // flipped_rl_disparity_ =
    //     vxCreateImage(context_, params.image_size.width,
    //                   params.image_size.height, VX_DF_IMAGE_S16);
    // VX_CHECK_STATUS(vxGetStatus((vx_reference)flipped_rl_disparity_));

    vx_node left_scale_node = vxScaleImageNode(
        graph_, left_image_, left_scaled_, VX_INTERPOLATION_BILINEAR);
    VX_CHECK_STATUS(vxVerifyGraph(graph_));
    vx_node right_scale_node = vxScaleImageNode(
        graph_, right_image_, right_scaled_, VX_INTERPOLATION_BILINEAR);
    VX_CHECK_STATUS(vxVerifyGraph(graph_));

    vx_node left_flip_node = nvxFlipImageNode(
        graph_, left_scaled_, flipped_left_, NVX_FLIP_HORIZONTAL);
    VX_CHECK_STATUS(vxVerifyGraph(graph_));

    vx_node right_flip_node = nvxFlipImageNode(
        graph_, right_scaled_, flipped_right_, NVX_FLIP_HORIZONTAL);
    VX_CHECK_STATUS(vxVerifyGraph(graph_));

    vx_node sgm_node = nvxSemiGlobalMatchingNode(
        graph_, left_scaled_, right_scaled_, disparity_scaled_,
        params.min_disparity, params.max_disparity, params.P1, params.P2,
        params.sad_win_size, params.ct_win_size, params.hc_win_size,
        params.clip, params.max_diff, params.uniqueness_ratio,
        params.scanline_mask, params.flags);
    VX_CHECK_STATUS(vxVerifyGraph(graph_));

    vx_node rl_sgm_node = nvxSemiGlobalMatchingNode(
        graph_, flipped_right_, flipped_left_, flipped_rl_disparity_scaled_,
        params.min_disparity, params.max_disparity, params.P1, params.P2,
        params.sad_win_size, params.ct_win_size, params.hc_win_size,
        params.clip, params.max_diff, params.uniqueness_ratio,
        params.scanline_mask, params.flags);
    VX_CHECK_STATUS(vxVerifyGraph(graph_));

    // vx_node disparity_flip_node =
    //     nvxFlipImageNode(graph_, flipped_rl_disparity_scaled_,
    //                      negated_rl_disparity_scaled_, NVX_FLIP_HORIZONTAL);
    // VX_CHECK_STATUS(vxVerifyGraph(graph_));

    // vx_node disparity_negate_node = nvxMultiplyByScalarNode(
    //     graph_, rl_disparity_scaled_, negated_rl_disparity_scaled_, -1);
    // VX_CHECK_STATUS(vxVerifyGraph(graph_));

    // vx_node disparity_scale_node = vxScaleImageNode(
    //     graph_, disparity_scaled_, disparity_, VX_INTERPOLATION_BILINEAR);
    // VX_CHECK_STATUS(vxVerifyGraph(graph_));

    // vx_node rl_disparity_scale_node =
    //     vxScaleImageNode(graph_, flipped_rl_disparity_scaled_,
    //                      flipped_rl_disparity_, VX_INTERPOLATION_BILINEAR);
    // VX_CHECK_STATUS(vxVerifyGraph(graph_));

    vxReleaseNode(&left_scale_node);
    vxReleaseNode(&right_scale_node);
    // vxReleaseNode(&left_flip_node);
    // vxReleaseNode(&right_flip_node);
    // vxReleaseNode(&disparity_scale_node);
    // vxReleaseNode(&rl_disparity_scale_node);
    // vxReleaseNode(&disparity_flip_node);
    // vxReleaseNode(&disparity_negate_node);
    vxReleaseNode(&sgm_node);
    vxReleaseNode(&rl_sgm_node);
  } else {
    ROS_INFO("min_disp %d, max_disp %d, P1 %d, P2 %d, SAD %d, CT %d, HC %d, "
             "clip %d, max_diff %d, UR %d, Scantype "
             "%02X, Flags %02X",
             params.min_disparity, params.max_disparity, params.P1, params.P2,
             params.sad_win_size, params.ct_win_size, params.hc_win_size,
             params.clip, params.max_diff, params.uniqueness_ratio,
             params.scanline_mask, params.flags);
    vx_node sgm_node = nvxSemiGlobalMatchingNode(
        graph_, left_image_, right_image_, disparity_, params.min_disparity,
        params.max_disparity, params.P1, params.P2, params.sad_win_size,
        params.ct_win_size, params.hc_win_size, params.clip, params.max_diff,
        params.uniqueness_ratio, params.scanline_mask, params.flags);
    VX_CHECK_STATUS(vxVerifyGraph(graph_));
    vxReleaseNode(&sgm_node);
  }
}

VXBidirectionalStereoMatcher::~VXBidirectionalStereoMatcher() {}


void VXBidirectionalStereoMatcher::setLambda (double _lambda) {
    _wls_params.lambda = _lambda;
}

void VXBidirectionalStereoMatcher::setLRCthresh (int _LRC_thresh) {
_wls_params.lrc_threshold = _LRC_thresh;
}


void VXBidirectionalStereoMatcher::compute(cv::InputArray left,
                                           cv::InputArray right,
                                           cv::OutputArray disparity) {
  copy_to_vx_image(left, left_image_);
  copy_to_vx_image(right, right_image_);

  const auto status = vxProcessGraph(graph_);
  ROS_ASSERT(status == VX_SUCCESS);

  if (params_.filtering == VXStereoMatcherParams::Filtering_WLS_LeftRight) {

    // This algorithm works on the **scaled** image
    // cv::Mat flipped_rl_disparity_mat, rl_disparity_mat, lr_disparity_mat;

    cv::Mat rl_disparity;

    nvx_cv::VXImageToCVMatMapper lr_disparity_map(
        disparity_scaled_, 0, NULL, VX_READ_ONLY, VX_MEMORY_TYPE_HOST);

    nvx_cv::VXImageToCVMatMapper right_map(right_scaled_, 0, NULL, VX_READ_ONLY,
                                           VX_MEMORY_TYPE_HOST);
    nvx_cv::VXImageToCVMatMapper left_map(left_scaled_, 0, NULL, VX_READ_ONLY,
                                          VX_MEMORY_TYPE_HOST);

    nvx_cv::VXImageToCVMatMapper flipped_rl_disparity_map(
        flipped_rl_disparity_scaled_, 0, NULL, VX_READ_ONLY,
        VX_MEMORY_TYPE_HOST);

    // Flip and negate RL disparities
    cv::flip(flipped_rl_disparity_map.getMat(), rl_disparity, 1);
    rl_disparity *= -1;

    // Since we don't use an OpenCV matcher, we need to create a fake
    // matcher to initialize the WLS filter
    //
    // These are the only params used to initialize the WLS filter...
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        params_.min_disparity, (params_.max_disparity - params_.min_disparity),
        params_.sad_win_size);
    cv ::Ptr<cv::ximgproc::DisparityWLSFilter> wls =
        cv::ximgproc::createDisparityWLSFilter(sgbm);
    wls->setLambda( _wls_params.lambda );
    wls->setLRCthresh( _wls_params.lrc_threshold);

    // Supply our own ROI otherwise it drops half of the image
    const int border = params_.max_disparity;
    const cv::Rect roi(border, 0, left_map.getMat().cols - 2 * border,
                       left_map.getMat().rows);
    wls->filter(lr_disparity_map.getMat(), left_map.getMat(), filter_output_,
                rl_disparity, roi, right_map.getMat());

    scaled_confidence_ = wls->getConfidenceMap();

    // And scale back up
    cv::Mat scaled_output;
    cv::resize(filter_output_, scaled_output, cv::Size(), params_.shrink_scale,
               params_.shrink_scale, cv::INTER_NEAREST);

    disparity.assign(scaled_output * params_.shrink_scale);
  } else {
    ROS_WARN("In VXBidirectionalStereoMatcher but not doing WLSFiltering;  "
             "this shouldn't happen.");
    // Not sure why you'd be here otherwise...
    copy_from_vx_image(disparity_, disparity);
    disparity.assign(disparity.getMat() * params_.shrink_scale);
  }
}
