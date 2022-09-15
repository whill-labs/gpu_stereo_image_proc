#pragma once

#include <sensor_msgs/CameraInfo.h>

sensor_msgs::CameraInfoPtr
scaleCameraInfo(const sensor_msgs::CameraInfoConstPtr &cam,
                float shrink_scale = 1.0);
