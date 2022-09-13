# gpu_stereo_image_proc 

## Overview

This package provides ROS wrapper for two CUDA implementations of Semi-Global (Block) Matching (i.e. SGM or SGBM):

* [NVIDIA VisionWorks](https://developer.nvidia.com/embedded/visionworks), and
* [Fixstars libSGM](https://github.com/fixstars/libSGM)

It has diverged significantly from [whill-lab's](https://github.com/whill-labs) [upstream package](https://github.com/whill-labs/gpu_stereo_image_proc).

As this package provides the baseline stereo capabilities on our [Trisect underwater trifocal sensor](https://trisect-perception-sensor.gitlab.io/), our primary development environment is the Jetson NX running Jetpack 4.4.x.   It also assumes our [OpenCV](https://gitlab.com/apl-ocean-engineering/jetson/buildopencv) and [ROS](https://gitlab.com/apl-ocean-engineering/jetson/buildros1) builds.

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
catkin build
# Note: libSGM is automatically pulled to gpu_stereo_image_proc/libSGM as an external project in CMakeLists
```

## Usage

### Basic Usage

Basic usage of this package (e.g. subscribed/published topics, node structure) is compatible with [ros-perception/image_pipeline/stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/melodic/stereo_image_proc). Refer to its [wiki](http://wiki.ros.org/stereo_image_proc?distro=melodic) for quick start.

### Nodelets

This package contains nodelets for creating disparity images from stereo:

* `gpu_stereo_image_proc/libsgm_disparity`:  Nodelet which wraps Fixstars libSGM.
* `gpu_stereo_image_proc/vx_disparity`:  Nodelet which wraps NVIDIA VisionWorks.

### Parameters

Most of the parameters can be configured via [dynamic reconfigure](http://wiki.ros.org/dynamic_reconfigure). These wiki pages describes configurable parameters.

- [Parameter (libSGM)](https://github.com/WHILL/gpu_stereo_image_proc/wiki/Parameter-(libSGM))
- [Parameter (VisionWorks)](https://github.com/WHILL/gpu_stereo_image_proc/wiki/Parameter-(VisionWorks))

## Licenses

This software maintain's the upstream package's [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause).

The underlying code of this package is forked from [ros-perception/image_pipeline/stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/melodic/stereo_image_proc) which is distibuted under the [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause).

This package includes [libSGM](https://github.com/fixstars/libSGM) which is distributed under the [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0).

This package depends on [VisionWorks](https://developer.nvidia.com/embedded/visionworks) which NVIDIA reserves all the copyrights of.
