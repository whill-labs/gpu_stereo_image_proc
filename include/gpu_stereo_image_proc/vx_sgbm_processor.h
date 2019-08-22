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
#ifndef GPU_STEREO_IMAGE_PROC_VX_SGBM_PROCESSOR_H
#define GPU_STEREO_IMAGE_PROC_VX_SGBM_PROCESSOR_H
#include "gpu_stereo_image_proc/sgbm_processor.h"
#include "gpu_stereo_image_proc/vx_stereo_matcher.h"

namespace gpu_stereo_image_proc
{
class VXStereoSGBMProcessor : public StereoSGBMProcessor
{
public:
  VXStereoSGBMProcessor() : max_diff_(16), hc_win_size_(1), ct_win_size_(0), clip_(31), shirink_scale_(1)
  {
    stereo_matcher_.reset(new VXStereoMatcher(image_size_.width, image_size_.height, shirink_scale_));
  }

  virtual void processDisparity(const cv::Mat& left_rect, const cv::Mat& right_rect,
                                const image_geometry::StereoCameraModel& model,
                                stereo_msgs::DisparityImage&             disparity) const;

  void applyConfig()
  {
    stereo_matcher_.reset(new VXStereoMatcher(image_size_.width, image_size_.height, shirink_scale_, uniqueness_ratio_,
                                              max_diff_, hc_win_size_, ct_win_size_, clip_, P2_, P1_, max_disparity_,
                                              min_disparity_));
  }

  int  getDisp12MaxDiff() const;
  void setDisp12MaxDiff(int max_diff);

  int  getHcWinSize() const;
  void setHcWinSize(int hc_win_size);

  int  getCtWinSize() const;
  void setCtWinSize(int ct_win_size);

  int  getClip() const;
  void setClip(int clip);

  int  getShrinkScale() const;
  void setShrinkScale(int shrink_scale);

private:
  std::shared_ptr<VXStereoMatcher> stereo_matcher_;

  int max_diff_;
  int hc_win_size_;
  int ct_win_size_;
  int clip_;
  int shirink_scale_;
};

inline int VXStereoSGBMProcessor::getDisp12MaxDiff() const
{
  return max_diff_;
}

inline void VXStereoSGBMProcessor::setDisp12MaxDiff(int max_diff)
{
  max_diff_ = max_diff;
}

inline int VXStereoSGBMProcessor::getHcWinSize() const
{
  return hc_win_size_;
}

inline void VXStereoSGBMProcessor::setHcWinSize(int hc_win_size)
{
  hc_win_size_ = hc_win_size;
}

inline int VXStereoSGBMProcessor::getCtWinSize() const
{
  return ct_win_size_;
}

inline void VXStereoSGBMProcessor::setCtWinSize(int ct_win_size)
{
  ct_win_size_ = ct_win_size;
}

inline int VXStereoSGBMProcessor::getClip() const
{
  return clip_;
}

inline void VXStereoSGBMProcessor::setClip(int clip)
{
  clip_ = clip;
}

inline int VXStereoSGBMProcessor::getShrinkScale() const
{
  return shirink_scale_;
}

inline void VXStereoSGBMProcessor::setShrinkScale(int shrink_scale)
{
  shirink_scale_ = shrink_scale;
}
}  // namespace gpu_stereo_image_proc

#endif