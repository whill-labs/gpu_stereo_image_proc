/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023  University of Washington
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
 *   * Neither the name of the University of Washington nor the names of its
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
#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <memory>
#include <opencv2/calib3d/calib3d.hpp>

#include "gpu_stereo_image_proc/camera_info_conversions.h"
#include "gpu_stereo_image_proc/msg_conversions.h"

namespace gpu_stereo_image_proc {
using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class DisparityNodeletBase : public nodelet::Nodelet {
 public:
  DisparityNodeletBase();

  void connectCb();

 protected:
  virtual void onInit();

  virtual bool hasSubscribers() const = 0;
  virtual int downsample() const = 0;

  // The "true" image callback passed to the Subscriber
  //
  // It performs common overhead:
  //    - initializing model_ and scaled_model_
  //
  // Then calls imageCallback(...)
  void imageCb(const ImageConstPtr &l_image_msg,
               const CameraInfoConstPtr &l_info_msg,
               const ImageConstPtr &r_image_msg,
               const CameraInfoConstPtr &r_info_msg);

  // Virtual callback used by derived classes
  virtual void imageCallback(const ImageConstPtr &l_image_msg,
                             const CameraInfoConstPtr &l_info_msg,
                             const ImageConstPtr &r_image_msg,
                             const CameraInfoConstPtr &r_info_msg) = 0;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo>
      ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  boost::mutex connect_mutex_;

  image_geometry::StereoCameraModel model_, scaled_model_;
};

}  // namespace gpu_stereo_image_proc
