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
#ifndef GPU_STEREO_IMAGE_PROC_SGBM_PROCESSOR_H
#define GPU_STEREO_IMAGE_PROC_SGBM_PROCESSOR_H
#include <image_geometry/stereo_camera_model.h>
#include <image_proc/processor.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>

namespace gpu_stereo_image_proc {
struct StereoImageSet {
  image_proc::ImageSet left;
  image_proc::ImageSet right;
  stereo_msgs::DisparityImage disparity;
  sensor_msgs::PointCloud points;
  sensor_msgs::PointCloud2 points2;
};

class StereoSGBMProcessor {
public:
  enum class ImageProcFlag {
    LEFT_MONO = 1 << 0,
    LEFT_RECT = 1 << 1,
    LEFT_COLOR = 1 << 2,
    LEFT_RECT_COLOR = 1 << 3,
    RIGHT_MONO = 1 << 4,
    RIGHT_RECT = 1 << 5,
    RIGHT_COLOR = 1 << 6,
    RIGHT_RECT_COLOR = 1 << 7,
    DISPARITY = 1 << 8,
    POINT_CLOUD = 1 << 9,
    POINT_CLOUD2 = 1 << 10,

    LEFT_ALL = LEFT_MONO | LEFT_RECT | LEFT_COLOR | LEFT_RECT_COLOR,
    RIGHT_ALL = RIGHT_MONO | RIGHT_RECT | RIGHT_COLOR | RIGHT_RECT_COLOR,
    STEREO_ALL = DISPARITY | POINT_CLOUD | POINT_CLOUD2,
    ALL = LEFT_ALL | RIGHT_ALL | STEREO_ALL
  };

  StereoSGBMProcessor()
      : image_size_(640, 480), min_disparity_(0), max_disparity_(128),
        disparity_range_(64), P1_(200), P2_(400), shrink_scale_(1) {}

  virtual ~StereoSGBMProcessor(){};

  cv::Size getImageSize() const { return image_size_; }
  virtual bool setImageSize(cv::Size image_size) = 0;

  int getInterpolation() const;
  void setInterpolation(int interp);

  virtual float getUniquenessRatio() const {};
  virtual bool setUniquenessRatio(float ratio) = 0;

  int getMinDisparity() const;
  virtual bool setMinDisparity(int min_d) = 0;

  int getMaxDisparity() const;
  virtual bool setMaxDisparity(int max_d);

  int getDisparityRange() const;
  bool setDisparityRange(int range);

  int getP1() const { return P1_; }
  void setP1(int P1) {
    ROS_INFO("%s, in %d", __func__, P1);
    P1_ = P1;
  }

  int getP2() const { return P2_; }
  void setP2(int P2) {
    ROS_INFO("%s, in %d", __func__, P2);
    P2_ = P2;
  }

  int getShrinkScale() const { return shrink_scale_; }
  void setShrinkScale(int shrink_scale) { shrink_scale_ = shrink_scale; }

  // bool process(const sensor_msgs::ImageConstPtr&        left_raw,
  //              const sensor_msgs::ImageConstPtr&        right_raw,
  //              const image_geometry::StereoCameraModel& model,
  //              StereoImageSet&                          output,
  //              ImageProcFlag                            flags) const;

  virtual cv::Mat_<int16_t>
  processDisparity(const cv::Mat &left_rect, const cv::Mat &right_rect,
                   const image_geometry::StereoCameraModel &model) const = 0;

  void disparityToDisparityImage(const cv::Mat_<int16_t> disparity16,
                                 const image_geometry::StereoCameraModel &model,
                                 stereo_msgs::DisparityImage &disparity) const;

  // void processPoints(const stereo_msgs::DisparityImage&       disparity,
  //                    const cv::Mat&                           color,
  //                    const std::string&                       encoding,
  //                    const image_geometry::StereoCameraModel& model,
  //                    sensor_msgs::PointCloud&                 points)
  //                    const;

  // void processPoints2(const stereo_msgs::DisparityImage&       disparity,
  //                     const cv::Mat&                           coloer,
  //                     const std::string&                       encoding,
  //                     const image_geometry::StereoCameraModel& model,
  //                     sensor_msgs::PointCloud2&                points)
  //                     const;

protected:
  // image_proc::Processor mono_processor_;
  // mutable cv::Mat_<int16_t> disparity16_;
  // mutable cv::Mat_<cv::Vec3f> dense_points_;

  cv::Size image_size_;
  int min_disparity_;
  int max_disparity_;
  int disparity_range_;
  int P1_;
  int P2_;

  int shrink_scale_;
};

// inline int StereoSGBMProcessor::getInterpolation() const {
//   return mono_processor_.interpolation_;
// }

// inline void StereoSGBMProcessor::setInterpolation(int interp) {
//   mono_processor_.interpolation_ = interp;
// }

inline int StereoSGBMProcessor::getMinDisparity() const {
  return min_disparity_;
}

inline int StereoSGBMProcessor::getMaxDisparity() const {
  return max_disparity_;
}
inline bool StereoSGBMProcessor::setMaxDisparity(int max_d) {
  ROS_INFO("%s, in %d", __func__, max_d);
  max_disparity_ = max_d;
  return true;
}

inline int StereoSGBMProcessor::getDisparityRange() const {
  return (max_disparity_ - min_disparity_);
}

inline bool StereoSGBMProcessor::setDisparityRange(int range) {
  ROS_INFO("%s, in %d", __func__, range);
  if (range < 0)
    return false;
  disparity_range_ = range;
  return StereoSGBMProcessor::setMaxDisparity(min_disparity_ + range);
}
inline StereoSGBMProcessor::ImageProcFlag
operator|(StereoSGBMProcessor::ImageProcFlag lhs,
          StereoSGBMProcessor::ImageProcFlag rhs) {
  using T = std::underlying_type_t<StereoSGBMProcessor::ImageProcFlag>;
  return static_cast<StereoSGBMProcessor::ImageProcFlag>(static_cast<T>(lhs) |
                                                         static_cast<T>(rhs));
}

inline StereoSGBMProcessor::ImageProcFlag
operator&(StereoSGBMProcessor::ImageProcFlag lhs,
          StereoSGBMProcessor::ImageProcFlag rhs) {
  using T = std::underlying_type_t<StereoSGBMProcessor::ImageProcFlag>;
  return static_cast<StereoSGBMProcessor::ImageProcFlag>(static_cast<T>(lhs) &
                                                         static_cast<T>(rhs));
}

inline StereoSGBMProcessor::ImageProcFlag &
operator|=(StereoSGBMProcessor::ImageProcFlag &lhs,
           StereoSGBMProcessor::ImageProcFlag rhs) {
  lhs = lhs | rhs;
  return lhs;
}

} // namespace gpu_stereo_image_proc

#endif /* GPU_STEREO_IMAGE_PROC_SGBM_PROCESSOR_H */
