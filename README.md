# gpu_stereo_image_proc [![CircleCI](https://circleci.com/gh/WHILL/gpu_stereo_image_proc.svg?style=svg&circle-token=95d28a14b516f20bef6e607e2c88d5a3f3bd92fe)](https://circleci.com/gh/WHILL/gpu_stereo_image_proc)

## Overview

This package provides ROS wrapper for CUDA implementations of Semi-Global (Block) Matching (i.e. SGM or SGBM). This package currently supports [Fixstars libSGM](https://github.com/fixstars/libSGM) and [NVIDIA VisionWorks](https://developer.nvidia.com/embedded/visionworks) as stereo-matching engine.

## Requirement

- [CUDA (compute capabilities >= 3.5)](https://developer.nvidia.com/cuda-downloads)
- [VisionWorks 1.6 or later](https://developer.nvidia.com/embedded/visionworks)
- OpenCV 3.2 or later
- CMake 3.1 or later
- ROS Melodic

## Installation

Only "Build from Source" option is provided. Simply pull the source and build the package on your machine.

```sh
cd <path-to-your-catkin-workspace>/src
git clone https://github.com/WHILL/gpu_stereo_image_proc.git
cd ..
catkin_make
# Note: libSGM is automatically pulled to gpu_stereo_image_proc/libSGM as CMake's external project.
```

If you want to build this package on a machine which does not have any GPU, CMake would not be able to detect CUDA architecture and build may fail. In this case, you can specify CUDA architecture by providing build options.

```sh
catkin_make -DAUTO_DETECT_ARCH=OFF -DCUDA_ARCH="<your-selection>"
# e.g. -DCUDA_ARCH="-arch=sm_72" for Jetson Xavier
```

## Usage

### Basic Usage

Basic usage of this package (e.g. subscribed/published topics, node structure) is compatible with [ros-perception/image_pipeline/stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/melodic/stereo_image_proc). Refer to its [wiki](http://wiki.ros.org/stereo_image_proc?distro=melodic) for quick start.

### Nodelets

This package contains nodelets for creating disparity images from stereo.

#### gpu_stereo_image_proc/libsgm_disparity

Nodelet which wraps Fixstars libSGM.

#### gpu_stereo_image_proc/vx_disparity

Nodelet which wraps NVIDIA VisionWorks.

### Parameters

Most of the parameters can be configured via [dynamic reconfigure](http://wiki.ros.org/dynamic_reconfigure). These wiki pages describes configurable parameters.

- [Parameter (libSGM)](https://github.com/WHILL/gpu_stereo_image_proc/wiki/Parameter-(libSGM))
- [Parameter (VisionWorks)](https://github.com/WHILL/gpu_stereo_image_proc/wiki/Parameter-(VisionWorks))

### Example: Compare disparity calculation results

This package provides an example launch file which enables you to see difference of three stereo-mathing implementations: OpenCV (CPU), libSGM (GPU) and VisionWorks (GPU).

```sh
# Launch your stereo camera
# Launch your nodelet manager
rosrun nodelet nodelet manager __name:=<nodename-of-your-manager> __ns:=<namespace-of-your-camera>

# Launch image processing nodelets
roslaunch gpu_stereo_image_proc comparison.launch manager:=<nodename-of-your-manager> __ns:=<namespace-of-your-camera>
```

Note: `disparity` and `points2` topics are remapped with `libsgm_` prefix (libSGM) and `vx_` prefix (VisionWorks).

### Demo Video

See https://youtu.be/whCAjrDg9_A

## Licenses

- This package is distributed under the [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause).
- The underlying code of this package is forked from [ros-perception/image_pipeline/stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/melodic/stereo_image_proc) which is distibuted under the [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause).
- This package includes [libSGM](https://github.com/fixstars/libSGM) which is distributed under the [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0).
- This package depends on [VisionWorks](https://developer.nvidia.com/embedded/visionworks) which NVIDIA reserves all the copyrights of.
