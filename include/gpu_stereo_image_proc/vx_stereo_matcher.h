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

  VXStereoMatcher(const int image_width, const int image_height, const int shrink_scale, const int min_disparity = 0,
                  const int max_disparity = 64, const int P1 = 8, const int P2 = 109, const int sad_win_size = 5,
                  const int ct_win_size = 0, const int hc_win_size = 1, const int clip = 31, const int max_diff = 16,
                  const int uniqueness_ratio = 50, enum nvx_scanline_e scanline_mask = NVX_SCANLINE_CROSS,
                  enum nvx_sgm_flags_e flags = NVX_SGM_PYRAMIDAL_STEREO);

  VXStereoMatcher(VXStereoMatcher&& obj);

  ~VXStereoMatcher();

  VXStereoMatcher& operator=(VXStereoMatcher&& obj);

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
