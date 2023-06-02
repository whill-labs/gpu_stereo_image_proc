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
#pragma once

#include <ros/ros.h>

#include <opencv2/core.hpp>
// #include <opencv2/core/cuda.hpp>

#include "gpu_stereo_image_proc/vpi/vpi_stereo_matcher_base.h"

namespace gpu_stereo_image_proc_vpi {

class VPIStereoMatcher : public VPIStereoMatcherBase {
 public:
  VPIStereoMatcher(const VPIStereoMatcherParams &params);

  virtual ~VPIStereoMatcher();

  void compute(cv::InputArray left, cv::InputArray right) override;

  const VPIStereoMatcherParams &params() const { return params_; }

  // cv::Mat scaledLeftRect() const { return vxImageToMatWrapper(left_scaled_);
  // }

  virtual cv::Mat disparity() const { return disparity_m_; }
  virtual cv::Mat confidence() const { return confidence8_m_; }

  // cv::Mat disparity() const override {
  //   if (params_.filtering == VPIStereoMatcherParams::Filtering_Bilateral) {
  //     // I suspect this is inefficient...
  //     cv::Mat out;
  //     g_filtered_.download(out);
  //     return out;
  //   } else {
  //     // Call the super
  //     return VPIStereoMatcherBase::disparity();
  //   }
  //}

 protected:
  VPIPayload stereo_payload_;
  VPIStream stream_;

  // Scaled images (equal to {left|right}_image_ if not scaling)
  VPIImage left_blurred_, right_blurred_, left_scaled_, right_scaled_;
  VPIImage confidence_;

  // Output images, these wrap Mats
  cv::Mat confidence8_m_, disparity_m_;
  VPIImage confidence8_, disparity_;

  VPIStereoMatcherParams params_;
  VPIStereoMatcher() = delete;

  // noncopyable
  VPIStereoMatcher(const VPIStereoMatcher &) = delete;
  VPIStereoMatcher &operator=(const VPIStereoMatcher &) = delete;
};

}  // namespace gpu_stereo_image_proc_vpi
