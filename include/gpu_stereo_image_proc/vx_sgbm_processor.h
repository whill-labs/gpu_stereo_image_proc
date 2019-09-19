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
#include <ros/ros.h>
#include "gpu_stereo_image_proc/sgbm_processor.h"
#include "gpu_stereo_image_proc/vx_stereo_matcher.h"

namespace gpu_stereo_image_proc
{
class VXStereoSGBMProcessor : public StereoSGBMProcessor
{
public:
  VXStereoSGBMProcessor()
    : max_diff_(16), hc_win_size_(1), ct_win_size_(0), clip_(31), shrink_scale_(1), uniqueness_ratio_(15)
  {
    stereo_matcher_.reset(new VXStereoMatcher(image_size_.width, image_size_.height));
  }

  virtual void processDisparity(const cv::Mat& left_rect, const cv::Mat& right_rect,
                                const image_geometry::StereoCameraModel& model,
                                stereo_msgs::DisparityImage&             disparity) const;

  void applyConfig()
  {
    ROS_INFO("===================================");
    ROS_INFO("image_size  : w %d, h %d", image_size_.width, image_size_.height);
    ROS_INFO("shrink_scale: %d", shrink_scale_);
    ROS_INFO("Uniqueness  : %d", uniqueness_ratio_);
    ROS_INFO("Max Diff    : %d", max_diff_);
    ROS_INFO("P1/P2       : P1 %d, P2, %d", P1_, P2_);
    ROS_INFO("Win Size    : SAD %d, CT %d, HC %d", sad_win_size_, ct_win_size_, hc_win_size_);
    ROS_INFO("Clip        : %d", clip_);
    ROS_INFO("Min/Max Disp: min %d, max %d", min_disparity_, max_disparity_);
    ROS_INFO("ScanType    : %02X", scanline_mask_);
    ROS_INFO("Flags       : %02X", flags_);
    ROS_INFO("===================================");
    stereo_matcher_.reset(new VXStereoMatcher(image_size_.width, image_size_.height, shrink_scale_, min_disparity_,
                                              max_disparity_, P1_, P2_, sad_win_size_, ct_win_size_, hc_win_size_,
                                              clip_, max_diff_, uniqueness_ratio_, scanline_mask_, flags_));
  }

  float getUniquenessRatio() const
  {
    return static_cast<float>(uniqueness_ratio_) * 0.01;
  }
  bool setUniquenessRatio(float ratio)
  {
    ROS_INFO("%s, in %f", __func__, ratio);
    if(ratio < 0.0 || ratio > 1.0)
      return false;
    uniqueness_ratio_ = static_cast<int>(ratio * 100.0);
    ROS_INFO("%s, set %d", __func__, uniqueness_ratio_);
    return true;
  }

  bool setMinDisparity(int min_d)
  {
    ROS_INFO("%s, in %d", __func__, min_d);
    if(min_d > getMaxDisparity())
      return false;
    min_disparity_ = min_d;
    ROS_INFO("%s, set %d", __func__, min_disparity_);
    return true;
  }

  bool setMaxDisparity(int max_d)
  {
    ROS_INFO("%s, in %d", __func__, max_d);
    if(max_d < getMinDisparity())
      return false;
    max_disparity_ = max_d;
    ROS_INFO("%s, set %d", __func__, max_disparity_);
    return true;
  }

  int getDisp12MaxDiff() const
  {
    return max_diff_;
  }
  void setDisp12MaxDiff(int max_diff)
  {
    ROS_INFO("%s, in %d", __func__, max_diff);
    max_diff_ = max_diff;
  }

  int getCorrelationWindowSize() const
  {
    return sad_win_size_;
  }
  void setCorrelationWindowSize(int sad_win_size)
  {
    ROS_INFO("%s, in %d", __func__, sad_win_size);
    sad_win_size_ = sad_win_size;
  }

  int getHcWinSize() const
  {
    return hc_win_size_;
  }
  void setHcWinSize(int hc_win_size)
  {
    ROS_INFO("%s, in %d", __func__, hc_win_size);
    hc_win_size_ = hc_win_size;
  }

  int getCtWinSize() const
  {
    return ct_win_size_;
  }
  void setCtWinSize(int ct_win_size)
  {
    ROS_INFO("%s, in %d", __func__, ct_win_size);
    ct_win_size_ = ct_win_size;
  }

  int getClip() const
  {
    return clip_;
  }
  void setClip(int clip)
  {
    ROS_INFO("%s, in %d", __func__, clip);
    clip_ = clip;
  }

  int getShrinkScale() const
  {
    return shrink_scale_;
  }
  void setShrinkScale(int shrink_scale)
  {
    ROS_INFO("%s, in %d", __func__, shrink_scale);
    shrink_scale_ = shrink_scale;
  }

  void setFlags(uint8_t flags)
  {
    flags_ = flags;
  }

  void setPathType(uint8_t scanline_mask)
  {
    scanline_mask_ = scanline_mask;
  }

private:
  std::shared_ptr<VXStereoMatcher> stereo_matcher_;

  int max_diff_;
  int sad_win_size_;
  int hc_win_size_;
  int ct_win_size_;
  int clip_;
  int shrink_scale_;
  int uniqueness_ratio_;
  int scanline_mask_;
  int flags_;
};

}  // namespace gpu_stereo_image_proc

#endif
