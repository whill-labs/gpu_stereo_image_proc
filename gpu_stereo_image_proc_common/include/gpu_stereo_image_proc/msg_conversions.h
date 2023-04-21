#pragma once

#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include <opencv2/core.hpp>

stereo_msgs::DisparityImagePtr disparityToDisparityImage(
    const sensor_msgs::ImageConstPtr &image,
    const cv::Mat_<int16_t> disparity16,
    const image_geometry::StereoCameraModel &model, int min_disparity,
    int max_disparity, int border, cv::InputArray mask = cv::Mat());
