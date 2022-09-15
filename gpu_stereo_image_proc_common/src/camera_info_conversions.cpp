#include <gpu_stereo_image_proc/camera_info_conversions.h>
#include <ros/ros.h>

sensor_msgs::CameraInfoPtr
scaleCameraInfo(const sensor_msgs::CameraInfoConstPtr &cam,
                float shrink_scale) {

  sensor_msgs::CameraInfoPtr out =
      boost::make_shared<sensor_msgs::CameraInfo>();

  out->header = cam->header;

  out->width = cam->width / shrink_scale;
  out->height = cam->height / shrink_scale;

  // These don't make a lot of sense after scaling
  //  out->distortion_model
  //  out->D

  out->K[0] = cam->K[0] / shrink_scale;
  out->K[2] = cam->K[2] / shrink_scale;
  out->K[4] = cam->K[4] / shrink_scale;
  out->K[5] = cam->K[5] / shrink_scale;
  out->K[8] = 1;

  out->R = cam->R;

  out->P[0] = cam->P[0] / shrink_scale;
  out->P[2] = cam->P[2] / shrink_scale;
  out->P[3] = cam->P[3] / shrink_scale;

  out->P[5] = cam->P[5] / shrink_scale;
  out->P[6] = cam->P[6] / shrink_scale;
  out->P[7] = cam->P[7] / shrink_scale;

  out->P[10] = 1;

  return out;
}
