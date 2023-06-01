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
#include "gpu_stereo_image_proc/visionworks/vx_image_scaler.h"

#include "gpu_stereo_image_proc/visionworks/vx_conversions.h"

namespace gpu_stereo_image_proc_visionworks {

VxImageScaler::VxImageScaler(unsigned int downsample_log2)
    : downsample_log2_(downsample_log2) {}

VxGaussianImageScaler::VxGaussianImageScaler(unsigned int downsample_log2)
    : VxImageScaler(downsample_log2), images_(downsample_log2) {
  ;
}

vx_image VxGaussianImageScaler::addToGraph(vx_context context, vx_graph graph,
                                           vx_image input) {
  // Retrieve size of input image
  vx_uint32 input_width, input_height;
  assert(VX_SUCCESS ==
         vxQueryImage(input, VX_IMAGE_WIDTH, &input_width, sizeof(vx_uint32)));
  assert(VX_SUCCESS == vxQueryImage(input, VX_IMAGE_HEIGHT, &input_height,
                                    sizeof(vx_uint32)));

  if (downsample_log2_ == 0) {
    output_size_.width = input_width;
    output_size_.height = input_height;
    return input;
  }

  vx_df_image input_format;
  assert(VX_SUCCESS == vxQueryImage(input, VX_IMAGE_FORMAT, &input_format,
                                    sizeof(vx_df_image)));

  uint32_t layer_width = input_width;
  uint32_t layer_height = input_height;

  const vx_int32 gaussian_kernel_size = 3;

  for (int i = 0; i < downsample_log2_; i++) {
    layer_width = (layer_width + 1) / 2;
    layer_height = (layer_height + 1) / 2;

    images_[i] =
        vxCreateImage(context, layer_width, layer_height, input_format);

    VX_CHECK_STATUS(vxGetStatus((vx_reference)images_[i]));

    if (i == 0) {
      vx_node scale_node = vxHalfScaleGaussianNode(graph, input, images_[0],
                                                   gaussian_kernel_size);
      vxReleaseNode(&scale_node);
    } else {
      vx_node scale_node = vxHalfScaleGaussianNode(
          graph, images_[i - 1], images_[i], gaussian_kernel_size);

      VX_CHECK_STATUS(vxVerifyGraph(graph));
      vxReleaseNode(&scale_node);
    }
  }

  output_size_.width = layer_width;
  output_size_.height = layer_height;

  return images_.back();
}

}  // namespace gpu_stereo_image_proc_visionworks
