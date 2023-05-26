/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, WHILL, Inc.
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

#include "gpu_stereo_image_proc/visionworks/vx_conversions.h"

using std::cout;
using std::endl;

void copy_to_vx_image(cv::InputArray src, vx_image dest) {
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

  const auto status = vxCopyImagePatch(dest, &rect, 0, &addr, src_mat.data,
                                       VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
  // ROS_ASSERT(status == VX_SUCCESS);
}
