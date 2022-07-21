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

VXBidirectionalStereoMatcher::VXBidirectionalStereoMatcher()
    : context_(nullptr), graph_(nullptr), left_image_(nullptr),
      right_image_(nullptr), left_scaled_(nullptr), right_scaled_(nullptr),
      disparity_scaled_(nullptr), disparity_(nullptr) {}

VXBidirectionalStereoMatcher::VXBidirectionalStereoMatcher(
    const int image_width, const int image_height, const int shrink_scale,
    const int min_disparity, const int max_disparity, const int P1,
    const int P2, const int sad_win_size, const int ct_win_size,
    const int hc_win_size, const int clip, const int max_diff,
    const int uniqueness_ratio, const int scanline_mask, const int flags)
    : context_(nullptr), graph_(nullptr), left_image_(nullptr),
      right_image_(nullptr), left_scaled_(nullptr), right_scaled_(nullptr),
      disparity_scaled_(nullptr), disparity_(nullptr) {
  vx_status status;

  context_ = vxCreateContext();
  VX_CHECK_STATUS(vxGetStatus((vx_reference)context_));

  graph_ = vxCreateGraph(context_);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)graph_));

  left_image_ =
      vxCreateImage(context_, image_width, image_height, VX_DF_IMAGE_U8);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)left_image_));
  right_image_ =
      vxCreateImage(context_, image_width, image_height, VX_DF_IMAGE_U8);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)right_image_));
  disparity_ =
      vxCreateImage(context_, image_width, image_height, VX_DF_IMAGE_S16);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)disparity_));

  if (shrink_scale > 1) {
    left_scaled_ = vxCreateImage(context_, image_width / shrink_scale,
                                 image_height / shrink_scale, VX_DF_IMAGE_U8);
    VX_CHECK_STATUS(vxGetStatus((vx_reference)left_scaled_));
    right_scaled_ = vxCreateImage(context_, image_width / shrink_scale,
                                  image_height / shrink_scale, VX_DF_IMAGE_U8);
    VX_CHECK_STATUS(vxGetStatus((vx_reference)right_scaled_));
    disparity_scaled_ =
        vxCreateImage(context_, image_width / shrink_scale,
                      image_height / shrink_scale, VX_DF_IMAGE_S16);
    VX_CHECK_STATUS(vxGetStatus((vx_reference)disparity_scaled_));

    vx_node left_scale_node = vxScaleImageNode(
        graph_, left_image_, left_scaled_, VX_INTERPOLATION_BILINEAR);
    VX_CHECK_STATUS(vxVerifyGraph(graph_));
    vx_node right_scale_node = vxScaleImageNode(
        graph_, right_image_, right_scaled_, VX_INTERPOLATION_BILINEAR);
    VX_CHECK_STATUS(vxVerifyGraph(graph_));

    vx_node sgm_node = nvxSemiGlobalMatchingNode(
        graph_, left_scaled_, right_scaled_, disparity_scaled_, min_disparity,
        max_disparity, P1, P2, sad_win_size, ct_win_size, hc_win_size, clip,
        max_diff, uniqueness_ratio, scanline_mask, flags);
    VX_CHECK_STATUS(vxVerifyGraph(graph_));
    vx_node disparity_scale_node = vxScaleImageNode(
        graph_, disparity_scaled_, disparity_, VX_INTERPOLATION_BILINEAR);
    VX_CHECK_STATUS(vxVerifyGraph(graph_));

    vxReleaseNode(&left_scale_node);
    vxReleaseNode(&right_scale_node);
    vxReleaseNode(&disparity_scale_node);
    vxReleaseNode(&sgm_node);
  } else {
    ROS_INFO("min_disp %d, max_disp %d, P1 %d, P2 %d, SAD %d, CT %d, HC %d, "
             "clip %d, max_diff %d, UR %d, Scantype "
             "%02X, Flags %02X",
             min_disparity, max_disparity, P1, P2, sad_win_size, ct_win_size,
             hc_win_size, clip, max_diff, uniqueness_ratio, scanline_mask,
             flags);
    vx_node sgm_node = nvxSemiGlobalMatchingNode(
        graph_, left_image_, right_image_, disparity_, min_disparity,
        max_disparity, P1, P2, sad_win_size, ct_win_size, hc_win_size, clip,
        max_diff, uniqueness_ratio, scanline_mask, flags);
    VX_CHECK_STATUS(vxVerifyGraph(graph_));
    vxReleaseNode(&sgm_node);
  }
}

VXBidirectionalStereoMatcher::VXBidirectionalStereoMatcher(
    VXBidirectionalStereoMatcher &&obj)
    : context_(obj.context_), graph_(obj.graph_), left_image_(obj.left_image_),
      right_image_(obj.right_image_), disparity_(obj.disparity_) {
  obj.context_ = 0;
  obj.graph_ = 0;
  obj.left_image_ = 0;
  obj.right_image_ = 0;
  obj.disparity_ = 0;
}

VXBidirectionalStereoMatcher::~VXBidirectionalStereoMatcher() {
  vxReleaseImage(&left_image_);
  vxReleaseImage(&right_image_);
  vxReleaseImage(&disparity_);
  vxReleaseGraph(&graph_);
  vxReleaseContext(&context_);
}

VXBidirectionalStereoMatcher &VXBidirectionalStereoMatcher::
operator=(VXBidirectionalStereoMatcher &&obj) {
  context_ = obj.context_;
  graph_ = obj.graph_;
  left_image_ = obj.left_image_;
  right_image_ = obj.right_image_;
  disparity_ = obj.disparity_;

  obj.context_ = 0;
  obj.graph_ = 0;
  obj.left_image_ = 0;
  obj.right_image_ = 0;
  obj.disparity_ = 0;

  return *this;
}

void VXBidirectionalStereoMatcher::compute(cv::InputArray left,
                                           cv::InputArray right,
                                           cv::OutputArray disparity) {
  copy_to_vx_image(left, left_image_);
  copy_to_vx_image(right, right_image_);

  const auto status = vxProcessGraph(graph_);
  ROS_ASSERT(status == VX_SUCCESS);

  copy_from_vx_image(disparity_, disparity);
}
