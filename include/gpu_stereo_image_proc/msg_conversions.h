#pragma once

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/core.hpp>
#include <stereo_msgs/DisparityImage.h>

void disparityToDisparityImage(const cv::Mat_<int16_t> disparity16,
                               const image_geometry::StereoCameraModel &model,
                               stereo_msgs::DisparityImage &disparity,
                               int min_disparity, int max_disparity);