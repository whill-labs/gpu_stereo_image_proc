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
#ifndef VX_STEREO_MATCHER_H
#define VX_STEREO_MATCHER_H

#include <opencv2/core.hpp>

#include <NVX/nvx.h>
#include <VX/vx.h>
#include <VX/vxu.h>

class VXStereoMatcher
{
public:
  VXStereoMatcher();

  VXStereoMatcher(const int image_width, const int image_height, const int shrink_scale = 2, const int min_disparity = 0,
                  const int max_disparity = 64, const int P1 = 8, const int P2 = 109, const int sad_win_size = 5,
                  const int ct_win_size = 0, const int hc_win_size = 1, const int clip = 31, const int max_diff = 16,
                  const int uniqueness_ratio = 50, enum nvx_scanline_e scanline_mask = NVX_SCANLINE_CROSS,
                  enum nvx_sgm_flags_e flags = NVX_SGM_PYRAMIDAL_STEREO);

  VXStereoMatcher(VXStereoMatcher&& obj);

  ~VXStereoMatcher();

  VXStereoMatcher& operator=(VXStereoMatcher&& obj);

  void compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity);

private:
  vx_context context_;
  vx_graph   graph_;
  vx_image   left_image_;
  vx_image   right_image_;
  vx_image   left_scaled_;
  vx_image   right_scaled_;
  vx_image   disparity_scaled_;
  vx_image   disparity_;

  // noncopyable
  VXStereoMatcher(const VXStereoMatcher&) = delete;
  VXStereoMatcher& operator=(const VXStereoMatcher&) = delete;
};

#endif
