#include <cv_bridge/cv_bridge.h>
#include <gpu_stereo_image_proc/msg_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

using namespace sensor_msgs;
using namespace stereo_msgs;

DisparityImageGenerator::DisparityImageGenerator(
    const image_geometry::StereoCameraModel &model, int min_disparity,
    int max_disparity, int border)
    : model_(model),
      min_disparity_(min_disparity),
      max_disparity_(max_disparity),
      border_(border) {}

DisparityImageResult DisparityImageGenerator::generate(
    const ImageConstPtr &image, const cv::Mat_<int16_t> disparity16) {
  return DisparityImageResult(image, disparity16, model_, min_disparity_,
                              max_disparity_, border_);
}

DisparityImageResult::DisparityImageResult(
    const ImageConstPtr &image, const cv::Mat_<int16_t> disparity16,
    const image_geometry::StereoCameraModel &model, int min_disparity,
    int max_disparity, int border) {
  disparity = boost::make_shared<DisparityImage>();
  disparity->header = image->header;
  disparity->image.header = image->header;

  const int DPP = 16;                // disparities per pixel
  const double inv_dpp = 1.0 / DPP;  // downsample / DPP

  // Fill in DisparityImage image data, converting to 32-bit float
  sensor_msgs::Image &dimage = disparity->image;
  dimage.height = disparity16.rows;
  dimage.width = disparity16.cols;
  dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  dimage.step = dimage.width * sizeof(float);
  dimage.data.resize(dimage.step * dimage.height);
  cv::Mat_<float> dmat(dimage.height, dimage.width, (float *)&dimage.data[0],
                       dimage.step);

  // We convert from fixed-point to float disparity and also adjust for any
  // x-offset between the principal points: d = d_fp*inv_dpp - (cx_l - cx_r)
  disparity16.convertTo(dmat, dmat.type(), inv_dpp,
                        -(model.left().cx() - model.right().cx()));
  ROS_ASSERT(dmat.data == &dimage.data[0]);
  /// @todo is_bigendian? :)

  // Find all points with disparity less than min_disparity
  bad_disparity_mask_ = cv::Mat(dmat.size(), CV_8UC1);
  cv::compare(dmat, min_disparity, bad_disparity_mask_, cv::CMP_LT);

  // Explicitly set those disparities to bad_disparity_value
  const float bad_disparity_value = 0.0;
  dmat.setTo(bad_disparity_value, bad_disparity_mask_);

  const int left = max_disparity + border - 1;
  const int wtf = (min_disparity >= 0) ? border + min_disparity
                                       : std::max(border, -min_disparity);
  const int right = disparity->image.width - 1 - wtf;
  const int top = border;
  const int bottom = disparity->image.height - 1 - border;
  cv::Rect valid_window(left, top, right - left, bottom - top);

  disparity->valid_window.x_offset = valid_window.x;
  disparity->valid_window.y_offset = valid_window.y;
  disparity->valid_window.width = valid_window.width;
  disparity->valid_window.height = valid_window.height;

  // Stereo parameters
  disparity->f = model.right().fx();
  disparity->T = model.baseline();

  /// @todo Window of (potentially) valid disparities

  // Disparity search range
  disparity->min_disparity = min_disparity;
  disparity->max_disparity = max_disparity;
  disparity->delta_d = inv_dpp;
}

stereo_msgs::DisparityImagePtr DisparityImageResult::getDisparity() {
  return disparity;
}

sensor_msgs::ImagePtr DisparityImageResult::getDepth() {
  ImagePtr depth_msg = boost::make_shared<Image>();

  sensor_msgs::Image &dimage = disparity->image;
  ROS_ASSERT((dimage.height > 0) && (dimage.width > 0));
  ROS_ASSERT(dimage.encoding == sensor_msgs::image_encodings::TYPE_32FC1);
  cv::Mat_<float> dmat(dimage.height, dimage.width,
                       reinterpret_cast<float *>(&dimage.data[0]), dimage.step);

  depth_msg->header = dimage.header;
  depth_msg->height = dimage.height;
  depth_msg->width = dimage.width;
  depth_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_msg->step = depth_msg->width * sizeof(float);
  depth_msg->data.resize(depth_msg->step * depth_msg->height);
  cv::Mat_<float> depth_mat(depth_msg->height, depth_msg->width,
                            reinterpret_cast<float *>(&(depth_msg->data[0])),
                            depth_msg->step);

  ROS_ASSERT((depth_mat.size().width == dmat.size().width) &&
             (depth_mat.size().height == dmat.size().height));

  const float numerator = (disparity->f * disparity->T);
  cv::divide(numerator, dmat, depth_mat);

  // Explicitly set all of the depths calculated from the bad disparities
  // to bad_depth_value
  const float bad_depth_value = std::numeric_limits<double>::quiet_NaN();
  depth_mat.setTo(bad_depth_value, bad_disparity_mask_);

  // Debug code to find and display the minimum and maximum disparities
  // {
  // cv::Mat zero_disparity_mask(dmat.size(), CV_8UC1);
  // cv::compare(dmat, 0, zero_disparity_mask, cv::CMP_NE);

  // double minVal,maxVal;
  // cv::minMaxLoc(depth_mat, &minVal, &maxVal, nullptr, nullptr,
  // zero_disparity_mask); ROS_INFO_STREAM("Min depth: " << minVal << "; max
  // depth: " << maxVal);
  // }

  return depth_msg;
}
