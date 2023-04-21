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

#include <NVX/nvx.h>
#include <VX/vx.h>
#include <VX/vxu.h>
#include <ros/ros.h>

#include <NVX/nvx_opencv_interop.hpp>
#include <opencv2/core.hpp>

#include "gpu_stereo_image_proc/visionworks/vx_stereo_matcher_params.h"

class VXStereoMatcherBase {
 public:
  VXStereoMatcherBase();

  VXStereoMatcherBase(const VXStereoMatcherParams &params);

  virtual ~VXStereoMatcherBase();

  virtual void compute(cv::InputArray left, cv::InputArray right) = 0;

  const VXStereoMatcherParams &params() const { return params_; }

  cv::Mat unfilteredDisparityMat() const {
    cv::Mat output;
    nvx_cv::VXImageToCVMatMapper map(disparity_, 0, NULL, VX_READ_ONLY,
                                     VX_MEMORY_TYPE_HOST);
    return map.getMat();
  }

  cv::Mat scaledLeftRect() const {
    if (params().downsample != 1) {
      nvx_cv::VXImageToCVMatMapper map(left_scaled_, 0, NULL, VX_READ_ONLY,
                                       VX_MEMORY_TYPE_HOST);
      return map.getMat();
    } else {
      nvx_cv::VXImageToCVMatMapper map(left_image_, 0, NULL, VX_READ_ONLY,
                                       VX_MEMORY_TYPE_HOST);
      return map.getMat();
    }
  }

  virtual cv::Mat disparity() const {
    nvx_cv::VXImageToCVMatMapper map(disparity_, 0, NULL, VX_READ_ONLY,
                                     VX_MEMORY_TYPE_HOST);
    return map.getMat();
  }

 protected:
  vx_context context_;
  vx_graph graph_;
  vx_image left_image_;
  vx_image right_image_;

  vx_image left_scaled_;
  vx_image right_scaled_;
  vx_image disparity_;

  VXStereoMatcherParams params_;

  // noncopyable
  VXStereoMatcherBase(const VXStereoMatcherBase &) = delete;
  VXStereoMatcherBase &operator=(const VXStereoMatcherBase &) = delete;
};
