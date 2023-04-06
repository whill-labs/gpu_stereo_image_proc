#pragma once

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

stereo_msgs::DisparityImagePtr
disparityToDisparityImage(const sensor_msgs::ImageConstPtr &image,
                          const cv::Mat_<int16_t> disparity16,
                          const image_geometry::StereoCameraModel &model,
                          int min_disparity, int max_disparity, int border,
                          float shrink_scale = 1.0);

sensor_msgs::ImagePtr 
disparityImageToDepthImage(DisparityImagePtr &disp_msg);