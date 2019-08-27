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
#ifndef GPU_STEREO_IMAGE_PROC_LIBSGM_SGBM_PROCESSOR_H
#define GPU_STEREO_IMAGE_PROC_LIBSGM_SGBM_PROCESSOR_H
#include "gpu_stereo_image_proc/sgbm_processor.h"
#include "libsgm.h"
#include "libsgm_wrapper.h"

namespace gpu_stereo_image_proc
{
class LibSGMStereoSGBMProcessor : public StereoSGBMProcessor
{
public:
  LibSGMStereoSGBMProcessor()
  {
    stereo_matcher_.reset(new sgm::LibSGMWrapper(max_disparity_, P1_, P2_, uniqueness_ratio_, true));
  }

  virtual void processDisparity(const cv::Mat& left_rect, const cv::Mat& right_rect,
                                const image_geometry::StereoCameraModel& model,
                                stereo_msgs::DisparityImage&             disparity) const;

  void applyConfig()
  {
    stereo_matcher_.reset(new sgm::LibSGMWrapper(max_disparity_, P1_, P2_, uniqueness_ratio_, true));
  }

  float getUniquenessRatio() const
  {
    return uniqueness_ratio_;
  }

  bool setUniquenessRatio(float ratio)
  {
    if(ratio < 0.0 || ratio > 1.0)
      return false;
    uniqueness_ratio_ = ratio;
    return true;
  }

  bool setMinDisparity(int min_d)
  {
    // min_disparity is fixed to zero in libSGM.
    // Setting minimum disparity is ignored.
    // See https://github.com/fixstars/libSGM/issues/36
    return false;
  }

  bool setMaxDisparity(int max_d)
  {
    if(max_d == 64 || max_d == 128)
    {
      max_disparity_ = max_d;
      return true;
    }
    return false;
  }

  int getDisp12MaxDiff() const
  {
    // Threshold for left-right consistency check is fixed to 1 in libSGM
    // See https://github.com/fixstars/libSGM/blob/master/src/check_consistency.cu
    return 1;
  }
  void setDisp12MaxDiff(int max_diff)
  {
  }

  int getCorrelationWindowSize() const
  {
    // Census window size is fixed to 9x7 in libSGM.
    // See https://github.com/fixstars/libSGM/issues/6
    return 9;
  }
  void setCorrelationWindowSize(int sad_win_size)
  {
  }

private:
  std::shared_ptr<sgm::LibSGMWrapper> stereo_matcher_;

  float uniqueness_ratio_;
};

}  // namespace gpu_stereo_image_proc

#endif
