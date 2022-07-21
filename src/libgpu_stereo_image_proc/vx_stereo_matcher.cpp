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
#include <opencv2/cudastereo.hpp>
#include <ros/ros.h>

#include "gpu_stereo_image_proc/visionworks/vx_stereo_matcher.h"

VXStereoMatcher::VXStereoMatcher() : VXStereoMatcherBase() {}

VXStereoMatcher::VXStereoMatcher(const VXStereoMatcherParams &params)
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

    vx_node left_scale_node = vxScaleImageNode(
        graph_, left_image_, left_scaled_, VX_INTERPOLATION_BILINEAR);
    VX_CHECK_STATUS(vxVerifyGraph(graph_));
    vx_node right_scale_node = vxScaleImageNode(
        graph_, right_image_, right_scaled_, VX_INTERPOLATION_BILINEAR);
    VX_CHECK_STATUS(vxVerifyGraph(graph_));

    vx_node sgm_node = nvxSemiGlobalMatchingNode(
        graph_, left_scaled_, right_scaled_, disparity_scaled_,
        params.min_disparity, params.max_disparity, params.P1, params.P2,
        params.sad_win_size, params.ct_win_size, params.hc_win_size,
        params.clip, params.max_diff, params.uniqueness_ratio,
        params.scanline_mask, params.flags);
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

VXStereoMatcher::~VXStereoMatcher() {}

// VXStereoMatcher &VXStereoMatcher::operator=(VXStereoMatcher &&obj) {
//   context_ = obj.context_;
//   graph_ = obj.graph_;
//   left_image_ = obj.left_image_;
//   right_image_ = obj.right_image_;
//   disparity_ = obj.disparity_;

//   obj.context_ = 0;
//   obj.graph_ = 0;
//   obj.left_image_ = 0;
//   obj.right_image_ = 0;
//   obj.disparity_ = 0;

//   return *this;
// }

void VXStereoMatcher::compute(cv::InputArray left, cv::InputArray right,
                              cv::OutputArray disparity) {
  copy_to_vx_image(left, left_image_);
  copy_to_vx_image(right, right_image_);

  const auto status = vxProcessGraph(graph_);
  ROS_ASSERT(status == VX_SUCCESS);

  copy_from_vx_image(disparity_, disparity);

  if (params_.filtering == VXStereoMatcherParams::Filtering_Bilateral) {
    // Do Bilateral filtering
    const int nDisp =
        (params_.max_disparity - params_.min_disparity) * params_.shrink_scale;
    const int radius = 3;
    const int iters = 1;

    cv::cuda::GpuMat disparity_in, left_img, disparity_out;
    disparity_in.upload(disparity);
    left_img.upload(left);

    cv::Ptr<cv::cuda::DisparityBilateralFilter> pCudaBilFilter =
        cv::cuda::createDisparityBilateralFilter(nDisp, radius, iters);

    pCudaBilFilter->apply(disparity_in, left_img, disparity_out);

    disparity_out.download(disparity);
  }
}
