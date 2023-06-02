#pragma once

#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include <opencv2/core.hpp>

class DisparityImageResult {
 public:
  DisparityImageResult(const sensor_msgs::ImageConstPtr &image,
                       const cv::Mat_<int16_t> disparity16,
                       const image_geometry::StereoCameraModel &model,
                       int min_disparity, int max_disparity, int border);

  stereo_msgs::DisparityImagePtr getDisparity();
  sensor_msgs::ImagePtr getDepth();

 private:
  stereo_msgs::DisparityImagePtr disparity;
  cv::Mat bad_disparity_mask_;

  DisparityImageResult() = delete;
  DisparityImageResult operator=(const DisparityImageResult &) = delete;
};

class DisparityImageGenerator {
 public:
  DisparityImageGenerator(const image_geometry::StereoCameraModel &model,
                          int min_disparity, int max_disparity, int border);

  DisparityImageResult generate(const sensor_msgs::ImageConstPtr &image,
                                const cv::Mat_<int16_t> disparity16);

 private:
  image_geometry::StereoCameraModel model_;
  int min_disparity_, max_disparity_, border_;

  DisparityImageGenerator() = delete;
  DisparityImageGenerator(const DisparityImageGenerator &) = delete;
  DisparityImageGenerator operator=(const DisparityImageGenerator &) = delete;
};

// Functional version for legacy purposes...
//
inline stereo_msgs::DisparityImagePtr disparityToDisparityImage(
    const sensor_msgs::ImageConstPtr &image,
    const cv::Mat_<int16_t> disparity16,
    const image_geometry::StereoCameraModel &model, int min_disparity,
    int max_disparity, int border) {
  DisparityImageGenerator dg(model, min_disparity, max_disparity, border);
  return dg.generate(image, disparity16).getDisparity();
}

// sensor_msgs::ImagePtr disparityImageToDepthImage(
//     const stereo_msgs::DisparityImageConstPtr &disp_msg);
