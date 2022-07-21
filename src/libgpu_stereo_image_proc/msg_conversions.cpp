#include <gpu_stereo_image_proc/msg_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

void disparityToDisparityImage(const cv::Mat_<int16_t> disparity16,
                               const image_geometry::StereoCameraModel &model,
                               stereo_msgs::DisparityImage &disparity,
                               int min_disparity, int max_disparity) {

  const int DPP = 16;               // disparities per pixel
  const double inv_dpp = 1.0 / DPP; // shrink_scale / DPP

  // Fill in DisparityImage image data, converting to 32-bit float
  sensor_msgs::Image &dimage = disparity.image;
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

  // Stereo parameters
  disparity.f = model.right().fx();
  disparity.T = model.baseline();

  /// @todo Window of (potentially) valid disparities

  // Disparity search range
  disparity.min_disparity = min_disparity;
  disparity.max_disparity = max_disparity;
  disparity.delta_d = inv_dpp;
}