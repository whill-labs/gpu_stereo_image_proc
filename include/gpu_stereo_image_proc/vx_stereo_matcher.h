#ifndef ROS_FISHEYE_STEREO_VX_STEREO_MATCHER_HPP
#define ROS_FISHEYE_STEREO_VX_STEREO_MATCHER_HPP

#include <opencv2/core.hpp>

#include <NVX/nvx.h>
#include <VX/vx.h>
#include <VX/vxu.h>

class VXStereoMatcher
{
public:
  VXStereoMatcher();

  VXStereoMatcher(int image_width, int image_height, int down_scale, const int uniqueness_ratio = 0,
                  const int max_diff = 16, const int hc_win_size = 1, const int ct_win_size = 0, const int clip = 31,
                  const int P2 = 109, const int P1 = 8, const int max_disparity = 64, const int min_disparity = 0);

  VXStereoMatcher(VXStereoMatcher&& obj);

  ~VXStereoMatcher();

  VXStereoMatcher& operator=(VXStereoMatcher&& obj);

  void operator()(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity);
  void compute(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity);

private:
  vx_context context_;
  vx_graph   graph_;
  vx_image   left_image_;
  vx_image   right_image_;
  vx_image   left_scaled_;
  vx_image   right_scaled_;
  vx_image   disparity_scaled_;
  vx_image   disparity_;

  // noncopyable
  VXStereoMatcher(const VXStereoMatcher&) = delete;
  VXStereoMatcher& operator=(const VXStereoMatcher&) = delete;
};

#endif
