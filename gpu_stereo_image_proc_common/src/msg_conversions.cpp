#include <gpu_stereo_image_proc/msg_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace sensor_msgs;
using namespace stereo_msgs;

DisparityImageGenerator::DisparityImageGenerator (
    const ImageConstPtr &image, const cv::Mat_<int16_t> disparity16,
    const image_geometry::StereoCameraModel &model, int min_disparity,
    int max_disparity, int border, float downsample) {

  DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
  disp_msg->header = image->header;
  disp_msg->image.header = image->header;

  const int DPP = 16;               // disparities per pixel
  const double inv_dpp = 1.0 / DPP; // downsample / DPP

  // Fill in DisparityImage image data, converting to 32-bit float
  sensor_msgs::Image &dimage = disp_msg->image;
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
                        -(model.left().cx() - model.right().cx()) /
                            downsample);
  ROS_ASSERT(dmat.data == &dimage.data[0]);
  /// @todo is_bigendian? :)

  const int left = max_disparity + border - 1;
  const int wtf = (min_disparity >= 0) ? border + min_disparity
                                       : std::max(border, -min_disparity);
  const int right = disp_msg->image.width - 1 - wtf;
  const int top = border;
  const int bottom = disp_msg->image.height - 1 - border;
  cv::Rect valid_window(left, top, right - left, bottom - top);

  disp_msg->valid_window.x_offset = valid_window.x;
  disp_msg->valid_window.y_offset = valid_window.y;
  disp_msg->valid_window.width = valid_window.width;
  disp_msg->valid_window.height = valid_window.height;

  // Stereo parameters
  disp_msg->f = model.right().fx() / downsample;
  disp_msg->T = model.baseline();

  /// @todo Window of (potentially) valid disparities

  // Disparity search range
  disp_msg->min_disparity = min_disparity;
  disp_msg->max_disparity = max_disparity;
  disp_msg->delta_d = inv_dpp;

  // Create the depth image
  ImagePtr depth_msg = boost::make_shared<Image>();
  depth_msg->header = disp_msg->image.header;
  depth_msg->height = dimage.height;
  depth_msg->width = dimage.width;
  depth_msg->encoding = dimage.encoding;
  ROS_ASSERT(depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1);
  /// @todo is_bigendian? :)
  depth_msg->step = dimage.step;
  depth_msg->data.resize(dimage.step * dimage.height);
  cv::Mat_<float> depth_mat(dimage.height, dimage.width, (float *)&depth_msg->data[0],
                       dimage.step);
  cv::divide((disp_msg->f * disp_msg->T), dmat, depth_mat);

  disparity = disp_msg;
  depth = depth_msg;
}

DisparityImageGenerator::getDisparity() {
  return disparity;
}

DisparityImageGenerator::getDepth() {
  return depth;
}

stereo_msgs::DisparityImagePtr disparityToDisparityImage(
    const ImageConstPtr &image, const cv::Mat_<int16_t> disparity16,
    const image_geometry::StereoCameraModel &model, int min_disparity,
    int max_disparity, int border, float downsample) {

  DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
  disp_msg->header = image->header;
  disp_msg->image.header = image->header;

  const int DPP = 16;               // disparities per pixel
  const double inv_dpp = 1.0 / DPP; // downsample / DPP

  // Fill in DisparityImage image data, converting to 32-bit float
  sensor_msgs::Image &dimage = disp_msg->image;
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
                        -(model.left().cx() - model.right().cx()) /
                            downsample);
  ROS_ASSERT(dmat.data == &dimage.data[0]);
  /// @todo is_bigendian? :)

  const int left = max_disparity + border - 1;
  const int wtf = (min_disparity >= 0) ? border + min_disparity
                                       : std::max(border, -min_disparity);
  const int right = disp_msg->image.width - 1 - wtf;
  const int top = border;
  const int bottom = disp_msg->image.height - 1 - border;
  cv::Rect valid_window(left, top, right - left, bottom - top);

  disp_msg->valid_window.x_offset = valid_window.x;
  disp_msg->valid_window.y_offset = valid_window.y;
  disp_msg->valid_window.width = valid_window.width;
  disp_msg->valid_window.height = valid_window.height;

  // Stereo parameters
  disp_msg->f = model.right().fx() / downsample;
  disp_msg->T = model.baseline();

  /// @todo Window of (potentially) valid disparities

  // Disparity search range
  disp_msg->min_disparity = min_disparity;
  disp_msg->max_disparity = max_disparity;
  disp_msg->delta_d = inv_dpp;

  return disp_msg;
}

sensor_msgs::ImagePtr disparityImageToDepthImage(const DisparityImageConstPtr &disp_msg) {

  // Create a deep copy of the disparity image
  ImagePtr depth_msg = boost::make_shared<Image>();
  depth_msg->header = disp_msg->image.header;
  depth_msg->height = disp_msg->image.height;
  depth_msg->width = disp_msg->image.width;
  depth_msg->encoding = disp_msg->image.encoding;
  depth_msg->is_bigendian = disp_msg->image.is_bigendian;
  depth_msg->step = disp_msg->image.step;
  depth_msg->data = disp_msg->image.data;

  // For disparity d, the depth from the camera is Z = fT/d.
  for (int row = 0; row < depth_msg->height; row++) {
    for (int step = 0; step < depth_msg->width; step++) {
      depth_msg->data[row * depth_msg->width + step] =
      disp_msg->f * disp_msg->T /
      depth_msg->data[row * depth_msg->width + step];
    }
  }

  return depth_msg;
}

// Adjust for any x-offset between the principal points: d' = d - (cx_l -
// cx_r)

// if (cx_l != cx_r) {
//   cv::Mat_<float> disp_image(
//       disp_msg->image.height, disp_msg->image.width,
//       reinterpret_cast<float *>(&disp_msg->image.data[0]),
//       disp_msg->image.step);
//   cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);
// }