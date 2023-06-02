/*********************************************************************
 *  Copyright (c) 2023 University of Washington
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>

#include <iostream>
// #include <opencv2/cudastereo.hpp>
// #include <opencv2/cudawarping.hpp>
#include <vpi/ImageFormat.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/GaussianFilter.h>
#include <vpi/algo/Rescale.h>
#include <vpi/algo/StereoDisparity.h>

#include <opencv2/imgproc.hpp>
#include <vpi/OpenCVInterop.hpp>

#include "gpu_stereo_image_proc/vpi/vpi_stereo_matcher.h"

namespace gpu_stereo_image_proc_vpi {

VPIStereoMatcher::VPIStereoMatcher(const VPIStereoMatcherParams &params)
    : params_(params) {
  // \todo ... fixed input type right now...
  const VPIImageFormat input_format = VPI_IMAGE_FORMAT_U8;

  const cv::Size sz = params.image_size();
  const cv::Size scaled_sz = params.scaled_image_size();

  vpiImageCreate(sz.width, sz.height, input_format, 0, &left_blurred_);
  vpiImageCreate(sz.width, sz.height, input_format, 0, &right_blurred_);

  vpiImageCreate(scaled_sz.width, scaled_sz.height, input_format, 0,
                 &left_scaled_);
  vpiImageCreate(scaled_sz.width, scaled_sz.height, input_format, 0,
                 &right_scaled_);

  confidence8_m_ = cv::Mat(scaled_sz, CV_8UC1);
  disparity_m_ = cv::Mat(scaled_sz, CV_16UC1);

  vpiImageCreateOpenCVMatWrapper(confidence8_m_, VPI_IMAGE_FORMAT_U8, 0,
                                 &confidence8_);
  vpiImageCreateOpenCVMatWrapper(disparity_m_, VPI_IMAGE_FORMAT_U16, 0,
                                 &disparity_);

  //  vpiImageCreate(scaled_sz.width, scaled_sz.height,
  //  VPI_IMAGE_FORMAT_S16, 0, &disparity);
  vpiImageCreate(scaled_sz.width, scaled_sz.height, VPI_IMAGE_FORMAT_U16, 0,
                 &confidence_);

  VPIStereoDisparityEstimatorCreationParams create_params;
  create_params.maxDisparity = params.max_disparity;

  vpiCreateStereoDisparityEstimator(VPI_BACKEND_CUDA, scaled_sz.width,
                                    scaled_sz.height, input_format,
                                    &create_params, &stereo_payload_);

  vpiStreamCreate(0, &stream_);
}

VPIStereoMatcher::~VPIStereoMatcher() {
  vpiStreamDestroy(stream_);
  vpiPayloadDestroy(stereo_payload_);

  vpiImageDestroy(left_scaled_);
  vpiImageDestroy(right_scaled_);

  vpiImageDestroy(left_blurred_);
  vpiImageDestroy(right_blurred_);
  vpiImageDestroy(disparity_);
  vpiImageDestroy(confidence_);
}

void VPIStereoMatcher::compute(cv::InputArray left_input,
                               cv::InputArray right_input) {
  VPIImage left, right;

  vpiImageCreateOpenCVMatWrapper(left_input.getMat(), 0, &left);
  vpiImageCreateOpenCVMatWrapper(right_input.getMat(), 0, &right);

  const int gaussian_filter_size = 5;
  const float gaussian_filter_sigma = 1.7;

  vpiSubmitGaussianFilter(stream_, VPI_BACKEND_CUDA, left, left_blurred_,
                          gaussian_filter_size, gaussian_filter_size,
                          gaussian_filter_sigma, gaussian_filter_sigma,
                          VPI_BORDER_ZERO);
  vpiSubmitGaussianFilter(stream_, VPI_BACKEND_CUDA, right, right_blurred_,
                          gaussian_filter_size, gaussian_filter_size,
                          gaussian_filter_sigma, gaussian_filter_sigma,
                          VPI_BORDER_ZERO);

  vpiSubmitRescale(stream_, VPI_BACKEND_CUDA, left_blurred_, left_scaled_,
                   VPI_INTERP_CATMULL_ROM, VPI_BORDER_ZERO, 0);
  vpiSubmitRescale(stream_, VPI_BACKEND_CUDA, right_blurred_, right_scaled_,
                   VPI_INTERP_CATMULL_ROM, VPI_BORDER_ZERO, 0);

  VPIStereoDisparityEstimatorParams stereo_params;
  stereo_params.windowSize = params_.window_size;
  stereo_params.maxDisparity = 0;  // Inherit from create_params_
  vpiSubmitStereoDisparityEstimator(stream_, VPI_BACKEND_CUDA, stereo_payload_,
                                    left_scaled_, right_scaled_, disparity_,
                                    confidence_, &stereo_params);

  // Scale the confidence to 0-255
  VPIConvertImageFormatParams cvtParams;
  vpiInitConvertImageFormatParams(&cvtParams);
  cvtParams.scale = 1.0f / 256;

  vpiSubmitConvertImageFormat(stream_, VPI_BACKEND_CUDA, confidence_,
                              confidence8_, &cvtParams);

  vpiStreamSync(stream_);

  vpiImageDestroy(left);
  vpiImageDestroy(right);

  // //  left_image_ = 	nvx_cv::createVXImageFromCVMat(context_, left.getMat());
  // //  right_image_ = 	nvx_cv::createVXImageFromCVMat(context_,
  // //  right.getMat());
  // copy_to_vx_image(left, left_image_);
  // copy_to_vx_image(right, right_image_);

  // const auto status = vxProcessGraph(graph_);
  // ROS_ASSERT(status == VX_SUCCESS);

  // if (params_.filtering == VPIStereoMatcherParams::Filtering_Bilateral) {
  //   const int nDisp = (params_.max_disparity - params_.min_disparity);
  //   const int radius = 3;
  //   const int iters = 1;

  //   nvx_cv::VXImageToCVMatMapper disparity_map(
  //       disparity_, 0, NULL, VX_READ_ONLY, NVX_MEMORY_TYPE_CUDA);
  //   nvx_cv::VXImageToCVMatMapper left_map(left_scaled_, 0, NULL,
  //   VX_READ_ONLY,
  //                                         NVX_MEMORY_TYPE_CUDA);

  //   cv::Ptr<cv::cuda::DisparityBilateralFilter> pCudaBilFilter =
  //       cv::cuda::createDisparityBilateralFilter(nDisp, radius, iters);

  //   pCudaBilFilter->apply(disparity_map.getGpuMat(), left_map.getGpuMat(),
  //                         g_filtered_);
  // }
}

}  // namespace gpu_stereo_image_proc_vpi
