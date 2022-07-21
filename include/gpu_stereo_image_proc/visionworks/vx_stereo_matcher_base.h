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
#pragma once

#include <opencv2/core.hpp>

#include <NVX/nvx.h>
#include <VX/vx.h>
#include <VX/vxu.h>
#include <ros/ros.h>

#include "gpu_stereo_image_proc/visionworks/vx_conversions.h"

struct VXStereoMatcherParams {
public:
  enum DisparityFiltering_t { Filtering_None = 0, Filtering_Bilateral = 1 };

  VXStereoMatcherParams()
      : shrink_scale(2), min_disparity(0), max_disparity(64), P1(8), P2(109),
        sad_win_size(5), ct_win_size(0), clip(31), max_diff(16),
        uniqueness_ratio(50), scanline_mask(NVX_SCANLINE_CROSS),
        flags(NVX_SGM_PYRAMIDAL_STEREO), filtering(Filtering_None) {}

  cv::Size image_size;
  int shrink_scale;
  int min_disparity;
  int max_disparity;

  int P1, P2, sad_win_size, ct_win_size, hc_win_size;
  int clip, max_diff, uniqueness_ratio, scanline_mask, flags;

  DisparityFiltering_t filtering;

  void dump() const {
    ROS_INFO("===================================");
    ROS_INFO("image_size  : w %d, h %d", image_size.width, image_size.height);
    ROS_INFO("shrink_scale: %d", shrink_scale);
    ROS_INFO("Uniqueness  : %d", uniqueness_ratio);
    ROS_INFO("Max Diff    : %d", max_diff);
    ROS_INFO("P1/P2       : P1 %d, P2, %d", P1, P2);
    ROS_INFO("Win Size    : SAD %d, CT %d, HC %d", sad_win_size, ct_win_size,
             hc_win_size);
    ROS_INFO("Clip        : %d", clip);
    ROS_INFO("Min/Max Disp: min %d, max %d", min_disparity, max_disparity);
    ROS_INFO("ScanType    : %02X", scanline_mask);
    ROS_INFO("Flags       : %02X", flags);
    ROS_INFO("Filtering   : %s", filter_as_string());
    ROS_INFO("===================================");
  }

  bool valid() const {
    if (image_size.width == 0 || image_size.height == 0)
      return false;

    return true;
  }

  const char *filter_as_string() const {
    if (filtering == Filtering_None) {
      return "None";
    } else if (filtering == Filtering_Bilateral) {
      return "Bilateral";
    }

    return "(Unknown)";
  }

  // Validations from vx_sgbm_processor which we could re-implement

  // if (image_size.width % 4 != 0) {
  //   ROS_WARN("Image Width must be divisible by 4.");
  //   return false;
  // }

  // if (ratio < 0.0 || ratio > 100.0)
  //   return false;
};

class VXStereoMatcherBase {
public:
  VXStereoMatcherBase();

  VXStereoMatcherBase(const VXStereoMatcherParams &params);

  // VXStereoMatcherBase(VXStereoMatcherBase &&obj);

  virtual ~VXStereoMatcherBase();

  //  VXStereoMatcher &operator=(VXStereoMatcher &&obj);

  virtual void compute(cv::InputArray left, cv::InputArray right,
                       cv::OutputArray disparity) = 0;

  const VXStereoMatcherParams &params() const { return params_; }

protected:
  vx_context context_;
  vx_graph graph_;
  vx_image left_image_;
  vx_image right_image_;
  vx_image left_scaled_;
  vx_image right_scaled_;
  vx_image disparity_scaled_;
  vx_image disparity_;

  VXStereoMatcherParams params_;

  // noncopyable
  VXStereoMatcherBase(const VXStereoMatcherBase &) = delete;
  VXStereoMatcherBase &operator=(const VXStereoMatcherBase &) = delete;
};
