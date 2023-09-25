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

#include "gpu_stereo_image_proc/vpi/vpi_stereo_matcher.h"

#include <ros/ros.h>
#include <vpi/ImageFormat.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/GaussianFilter.h>
#include <vpi/algo/Rescale.h>
#include <vpi/algo/StereoDisparity.h>

#include <iostream>
#include <opencv2/imgproc.hpp>
#include <vpi/OpenCVInterop.hpp>

#include "gpu_stereo_image_proc/vpi/vpi_status.h"

namespace gpu_stereo_image_proc_vpi {

VPIStereoMatcher::VPIStereoMatcher(const VPIStereoMatcherParams &params)
    : params_(params) {
  // \todo ... fixed input type right now...
  VPIImageFormat input_format;  //= VPI_IMAGE_FORMAT_U8;
  if (params_.image_type() == CV_16UC1) {
    ROS_INFO("Using 16-bit input images");
    input_format = VPI_IMAGE_FORMAT_U16;
  } else {
    ROS_INFO("Using 8-bit input images");
    input_format = VPI_IMAGE_FORMAT_U8;
  }

  const cv::Size sz = params_.image_size();
  const cv::Size scaled_sz = params_.scaled_image_size();

  VPI_CHECK_STATUS(
      vpiImageCreate(sz.width, sz.height, input_format, 0, &left_blurred_));

  VPI_CHECK_STATUS(
      vpiImageCreate(sz.width, sz.height, input_format, 0, &right_blurred_));

  ROS_INFO_STREAM(" Input image: " << sz.width << " x " << sz.height);
  ROS_INFO_STREAM("Scaled image: " << scaled_sz.width << " x "
                                   << scaled_sz.height);

  VPI_CHECK_STATUS(vpiImageCreate(scaled_sz.width, scaled_sz.height,
                                  input_format, 0, &left_scaled_));

  VPI_CHECK_STATUS(vpiImageCreate(scaled_sz.width, scaled_sz.height,
                                  input_format, 0, &right_scaled_));

  confidence8_m_ = cv::Mat(scaled_sz, CV_8UC1);

  VPI_CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(
      confidence8_m_, VPI_IMAGE_FORMAT_U8, 0, &confidence8_));

  // vpiImageCreateOpenCVMatWrapper(disparity_m_, VPI_IMAGE_FORMAT_U16, 0,
  //                                &disparity_);

  VPI_CHECK_STATUS(vpiImageCreate(scaled_sz.width, scaled_sz.height,
                                  VPI_IMAGE_FORMAT_U16, 0, &disparity_));

  VPI_CHECK_STATUS(vpiImageCreate(scaled_sz.width, scaled_sz.height,
                                  VPI_IMAGE_FORMAT_U16, 0, &confidence_));

  ROS_INFO_STREAM("Max disparity: " << params_.max_disparity);
  VPIStereoDisparityEstimatorCreationParams create_params;
  create_params.maxDisparity = params_.max_disparity;
  create_params.includeDiagonals = 1;

  VPI_CHECK_STATUS(vpiCreateStereoDisparityEstimator(
      VPI_BACKEND_CUDA, scaled_sz.width, scaled_sz.height, input_format,
      &create_params, &stereo_payload_));

  VPI_CHECK_STATUS(vpiStreamCreate(VPI_BACKEND_CUDA, &stream_));
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

  VPI_CHECK_STATUS(
      vpiImageCreateWrapperOpenCVMat(left_input.getMat(), 0, &left));
  VPI_CHECK_STATUS(
      vpiImageCreateWrapperOpenCVMat(right_input.getMat(), 0, &right));

  const int gaussian_filter_size = 5;
  const float gaussian_filter_sigma = 1.7;

  VPI_CHECK_STATUS(vpiSubmitGaussianFilter(
      stream_, VPI_BACKEND_CUDA, left, left_blurred_, gaussian_filter_size,
      gaussian_filter_size, gaussian_filter_sigma, gaussian_filter_sigma,
      VPI_BORDER_ZERO));
  VPI_CHECK_STATUS(vpiSubmitGaussianFilter(
      stream_, VPI_BACKEND_CUDA, right, right_blurred_, gaussian_filter_size,
      gaussian_filter_size, gaussian_filter_sigma, gaussian_filter_sigma,
      VPI_BORDER_ZERO));

  VPI_CHECK_STATUS(vpiSubmitRescale(stream_, VPI_BACKEND_CUDA, left_blurred_,
                                    left_scaled_, VPI_INTERP_CATMULL_ROM,
                                    VPI_BORDER_ZERO, 0));
  VPI_CHECK_STATUS(vpiSubmitRescale(stream_, VPI_BACKEND_CUDA, right_blurred_,
                                    right_scaled_, VPI_INTERP_CATMULL_ROM,
                                    VPI_BORDER_ZERO, 0));

  ///
  ///
  ///

  // VPIStereoDisparityEstimatorCreationParams create_params;
  // create_params.maxDisparity = 64; //params.max_disparity;

  // int32_t stereo_width, stereo_height;
  // VPIImageFormat stereo_format;
  // VPI_CHECK_STATUS(vpiImageGetSize( left_scaled_, &stereo_width,
  // &stereo_height )); VPI_CHECK_STATUS(vpiImageGetFormat(left_scaled_,
  // &stereo_format));

  // VPIPayload stereo_payload;
  // VPI_CHECK_STATUS(vpiCreateStereoDisparityEstimator(
  //     VPI_BACKEND_CUDA, stereo_width, stereo_height, stereo_format,
  //     &create_params, &stereo_payload));

  VPIStereoDisparityEstimatorParams stereo_params;
  stereo_params.windowSize =
      5;  // n.b. this is ignored on CUDA which uses 9x7 kernel
  stereo_params.maxDisparity = params_.max_disparity;
  stereo_params.confidenceType = VPI_STEREO_CONFIDENCE_ABSOLUTE;
  stereo_params.p1 = params_.p1;
  stereo_params.p2 = params_.p2;
  stereo_params.uniqueness = params_.uniqueness;

  // In the current generation of VPI, confidence is not
  // calculated.  This is fixed in later versions.
  VPI_CHECK_STATUS(vpiSubmitStereoDisparityEstimator(
      stream_, 0, stereo_payload_, left_scaled_, right_scaled_, disparity_,
      NULL, &stereo_params));

  // Scale the confidence to 0-255
  // VPIConvertImageFormatParams cvtParams;
  // vpiInitConvertImageFormatParams(&cvtParams);
  // cvtParams.scale = 1.0f / 256;

  // VPI_CHECK_STATUS(vpiSubmitConvertImageFormat(
  //     stream_, VPI_BACKEND_CUDA, confidence_, confidence8_, &cvtParams));

  VPI_CHECK_STATUS(vpiStreamSync(stream_));

  cv::Mat dispMat;
  VPIImageData disparityData;
  vpiImageLockData(disparity_, VPI_LOCK_READ,
                   VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR, &disparityData);
  vpiImageDataExportOpenCVMat(disparityData, &dispMat);

  double mmin, mmax;
  cv::minMaxLoc(dispMat, &mmin, &mmax);
  ROS_INFO_STREAM_THROTTLE(1, "Disparity min " << mmin << "; max = " << mmax);

  // ...
  dispMat.copyTo(disparity_m_);

  vpiImageUnlock(disparity_);

  vpiImageDestroy(left);
  vpiImageDestroy(right);

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
