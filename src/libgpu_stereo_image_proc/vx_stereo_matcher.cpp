#include <ros/ros.h>

#include "stereo_image_proc/vx_stereo_matcher.hpp"

#define VX_CHECK_STATUS(s)                                                                                             \
  do                                                                                                                   \
  {                                                                                                                    \
    const auto status = (s);                                                                                           \
    if (status != VX_SUCCESS)                                                                                          \
    {                                                                                                                  \
      ROS_ERROR("VX ERROR: %d", status);                                                                               \
    }                                                                                                                  \
    ROS_ASSERT(status == VX_SUCCESS);                                                                                  \
  } while (false)

namespace
{
void copy_to_vx_image(vx_image dest, cv::InputArray src)
{
  cv::Mat src_mat = src.getMat();
  const auto width = static_cast<vx_uint32>(src_mat.cols);
  const auto height = static_cast<vx_uint32>(src_mat.rows);

  vx_imagepatch_addressing_t addr;
  addr.dim_x = width;
  addr.dim_y = height;
  addr.stride_x = src_mat.elemSize();
  addr.stride_y = src_mat.step;
  addr.scale_x = VX_SCALE_UNITY;
  addr.scale_y = VX_SCALE_UNITY;
  addr.step_x = 1;
  addr.step_y = 1;

  vx_rectangle_t rect;
  rect.start_x = 0;
  rect.start_y = 0;
  rect.end_x = width;
  rect.end_y = height;

  const auto status = vxCopyImagePatch(dest, &rect, 0, &addr, src_mat.data, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
  ROS_ASSERT(status == VX_SUCCESS);
}

void copy_from_vx_image(cv::OutputArray dest, vx_image src)
{
  vx_uint32 width, height;
  vxQueryImage(src, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(width));
  vxQueryImage(src, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));

  vx_df_image vx_format = VX_DF_IMAGE_VIRT;
  vxQueryImage(src, VX_IMAGE_ATTRIBUTE_FORMAT, &vx_format, sizeof(vx_format));
  int cv_format = 0;
  if (vx_format == VX_DF_IMAGE_U8)
  {
    cv_format = CV_8U;
  }
  else if (vx_format == VX_DF_IMAGE_S16)
  {
    cv_format = CV_16S;
  }
  else
  {
    ROS_ERROR("unsupported iamge format");
  }

  dest.create(static_cast<int>(height), static_cast<int>(width), cv_format);
  cv::Mat dest_mat = dest.getMat();

  vx_imagepatch_addressing_t addr;
  addr.dim_x = width;
  addr.dim_y = height;
  addr.stride_x = dest_mat.elemSize();
  addr.stride_y = dest_mat.step;
  addr.scale_x = VX_SCALE_UNITY;
  addr.scale_y = VX_SCALE_UNITY;
  addr.step_x = 1;
  addr.step_y = 1;

  vx_rectangle_t rect;
  rect.start_x = 0;
  rect.start_y = 0;
  rect.end_x = width;
  rect.end_y = height;

  const auto status = vxCopyImagePatch(src, &rect, 0, &addr, dest_mat.data, VX_READ_ONLY, VX_MEMORY_TYPE_HOST);
  ROS_ASSERT(status == VX_SUCCESS);
}

}  // namespace

VXStereoMatcher::VXStereoMatcher()
  : context_(nullptr)
  , graph_(nullptr)
  , left_image_(nullptr)
  , right_image_(nullptr)
  , left_scaled_(nullptr)
  , right_scaled_(nullptr)
  , disparity_scaled_(nullptr)
  , disparity_(nullptr)
{
}

VXStereoMatcher::VXStereoMatcher(int image_width, int image_height, int down_scale, const int uniqueness_ratio,
                                 const int max_diff, const int hc_win_size, const int ct_win_size, const int clip,
                                 const int P2, const int P1, const int max_disparity, const int min_disparity)
  : context_(nullptr)
  , graph_(nullptr)
  , left_image_(nullptr)
  , right_image_(nullptr)
  , left_scaled_(nullptr)
  , right_scaled_(nullptr)
  , disparity_scaled_(nullptr)
  , disparity_(nullptr)
{
  vx_status status;

  context_ = vxCreateContext();
  VX_CHECK_STATUS(vxGetStatus((vx_reference)context_));

  graph_ = vxCreateGraph(context_);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)graph_));

  left_image_ = vxCreateImage(context_, image_width, image_height, VX_DF_IMAGE_U8);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)left_image_));
  right_image_ = vxCreateImage(context_, image_width, image_height, VX_DF_IMAGE_U8);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)right_image_));
  left_scaled_ = vxCreateImage(context_, image_width / down_scale, image_height / down_scale, VX_DF_IMAGE_U8);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)left_scaled_));
  right_scaled_ = vxCreateImage(context_, image_width / down_scale, image_height / down_scale, VX_DF_IMAGE_U8);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)right_scaled_));
  disparity_scaled_ = vxCreateImage(context_, image_width / down_scale, image_height / down_scale, VX_DF_IMAGE_S16);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)disparity_scaled_));
  disparity_ = vxCreateImage(context_, image_width, image_height, VX_DF_IMAGE_S16);
  VX_CHECK_STATUS(vxGetStatus((vx_reference)disparity_));

  vx_node left_scale_node = vxScaleImageNode(graph_, left_image_, left_scaled_, VX_INTERPOLATION_BILINEAR);
  VX_CHECK_STATUS(vxVerifyGraph(graph_));
  vx_node right_scale_node = vxScaleImageNode(graph_, right_image_, right_scaled_, VX_INTERPOLATION_BILINEAR);
  VX_CHECK_STATUS(vxVerifyGraph(graph_));

  const int sad = 5;

  vx_node sgm_node = nvxSemiGlobalMatchingNode(graph_, left_scaled_, right_scaled_, disparity_scaled_, min_disparity,
                                               max_disparity, P1, P2, sad, ct_win_size, hc_win_size, clip, max_diff,
                                               uniqueness_ratio, NVX_SCANLINE_ALL, NVX_SGM_PYRAMIDAL_STEREO);
  ROS_ASSERT(sgm_node);

  // const auto verify_status = vxVerifyGraph(graph_);
  // VX_CHECK_STATUS(verify_status);
  VX_CHECK_STATUS(vxVerifyGraph(graph_));

  vx_node disparity_scale_node = vxScaleImageNode(graph_, disparity_scaled_, disparity_, VX_INTERPOLATION_BILINEAR);
  VX_CHECK_STATUS(vxVerifyGraph(graph_));

  vxReleaseNode(&left_scale_node);
  vxReleaseNode(&right_scale_node);
  vxReleaseNode(&disparity_scale_node);
  vxReleaseNode(&sgm_node);
}

VXStereoMatcher::VXStereoMatcher(VXStereoMatcher&& obj)
  : context_(obj.context_)
  , graph_(obj.graph_)
  , left_image_(obj.left_image_)
  , right_image_(obj.right_image_)
  , disparity_(obj.disparity_)
{
  obj.context_ = 0;
  obj.graph_ = 0;
  obj.left_image_ = 0;
  obj.right_image_ = 0;
  obj.disparity_ = 0;
}

VXStereoMatcher::~VXStereoMatcher()
{
  vxReleaseImage(&left_image_);
  vxReleaseImage(&right_image_);
  vxReleaseImage(&disparity_);
  vxReleaseGraph(&graph_);
  vxReleaseContext(&context_);
}

VXStereoMatcher& VXStereoMatcher::operator=(VXStereoMatcher&& obj)
{
  context_ = obj.context_;
  graph_ = obj.graph_;
  left_image_ = obj.left_image_;
  right_image_ = obj.right_image_;
  disparity_ = obj.disparity_;

  obj.context_ = 0;
  obj.graph_ = 0;
  obj.left_image_ = 0;
  obj.right_image_ = 0;
  obj.disparity_ = 0;

  return *this;
}

void VXStereoMatcher::operator()(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
{
  copy_to_vx_image(left_image_, left);
  copy_to_vx_image(right_image_, right);

  const auto status = vxProcessGraph(graph_);
  ROS_ASSERT(status == VX_SUCCESS);

  copy_from_vx_image(disparity, disparity_);
}
