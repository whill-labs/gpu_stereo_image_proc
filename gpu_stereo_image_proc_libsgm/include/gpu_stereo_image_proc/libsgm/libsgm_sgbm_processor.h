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

namespace gpu_stereo_image_proc {
class LibSGMStereoSGBMProcessor : public StereoSGBMProcessor {
 public:
  LibSGMStereoSGBMProcessor() {
    stereo_matcher_.reset(new sgm::LibSGMWrapper(disparity_range_, P1_, P2_,
                                                 uniqueness_ratio_, true));
  }

  cv::Mat_<int16_t> processDisparity(
      const cv::Mat &left_rect, const cv::Mat &right_rect,
      const image_geometry::StereoCameraModel &model) const override;

  void applyConfig() {
    ROS_INFO("===================================");
    ROS_INFO("Uniqueness  : %5.1f", uniqueness_ratio_);
    ROS_INFO("P1/P2       : P1 %d, P2, %d", P1_, P2_);
    ROS_INFO("Min/Max Disp: min %d, max %d", min_disparity_, max_disparity_);
    ROS_INFO("Path Type   : %s",
             (path_type_ == sgm::PathType::SCAN_4PATH ? "SCAN_4PATH"
                                                      : "SCAN_8PATH"));
    ROS_INFO("===================================");
    stereo_matcher_.reset(new sgm::LibSGMWrapper(disparity_range_, P1_, P2_,
                                                 uniqueness_ratio_, true,
                                                 path_type_, min_disparity_));
  }

  bool setImageSize(cv::Size image_size) {
    ROS_WARN("Member variable 'image_size_' is not currently used.");
    image_size_ = image_size;
    return true;
  }

  float getUniquenessRatio() const {
    return (100.0 - uniqueness_ratio_) * 100.0;
  }

  bool setUniquenessRatio(float ratio) {
    if (ratio < 0.0 || ratio > 100.0) return false;
    uniqueness_ratio_ = (100.0 - ratio) / 100.0;
    return true;
  }

  bool setMinDisparity(int min_d) {
    min_disparity_ = min_d;
    return true;
  }

  bool setMaxDisparity(int max_d) {
    max_disparity_ = max_d;
    return true;
  }

  int getCorrelationWindowSize() const {
    // Census window size is fixed to 9x7 in libSGM.
    // See https://github.com/fixstars/libSGM/issues/6
    return 9;
  }

  void setCorrelationWindowSize(int sad_win_size) {}

  int getPathType() const {
    switch (path_type_) {
      case sgm::PathType::SCAN_4PATH:
        return 0;
        break;
      case sgm::PathType::SCAN_8PATH:
        return 1;
        break;
      default:
        return -1;
        break;
    }
  }

  bool setPathType(int path_type) {
    bool ret = true;
    switch (path_type) {
      case 0:
        path_type_ = sgm::PathType::SCAN_4PATH;
        break;
      case 1:
        path_type_ = sgm::PathType::SCAN_8PATH;
        break;
      default:
        ret = false;
        break;
    }
    return ret;
  }

 private:
  std::shared_ptr<sgm::LibSGMWrapper> stereo_matcher_;

  float uniqueness_ratio_;
  sgm::PathType path_type_;
};

}  // namespace gpu_stereo_image_proc

#endif
