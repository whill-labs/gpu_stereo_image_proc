#include <gpu_stereo_image_proc/msg_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

using namespace sensor_msgs;
using namespace stereo_msgs;

stereo_msgs::DisparityImagePtr disparityToDisparityImage(
    const ImageConstPtr &image, const cv::Mat_<int16_t> disparity16,
    const image_geometry::StereoCameraModel &model, int min_disparity,
    int max_disparity, int border, cv::InputArray mask) {
  DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
  disp_msg->header = image->header;
  disp_msg->image.header = image->header;

  const int DPP = 16;                // disparities per pixel
  const double inv_dpp = 1.0 / DPP;  // downsample / DPP

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
  if (!mask.empty()) {
    cv::Mat masked_disparity;
    disparity16.copyTo(masked_disparity, mask);

    // \todo(amarburg)  Interestingly there doesn't seem to be a masked
    // convertTo function..  could write one?
    disparity16.convertTo(dmat, dmat.type(), inv_dpp,
                          -(model.left().cx() - model.right().cx()));
  } else {
    disparity16.convertTo(dmat, dmat.type(), inv_dpp,
                          -(model.left().cx() - model.right().cx()));
  }
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
  disp_msg->f = model.right().fx();
  disp_msg->T = model.baseline();

  /// @todo Window of (potentially) valid disparities

  // Disparity search range
  disp_msg->min_disparity = min_disparity;
  disp_msg->max_disparity = max_disparity;
  disp_msg->delta_d = inv_dpp;

  return disp_msg;
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
