# gpu_stereo_image_proc 

## Overview

This package provides a set of ROS nodelets for CUDA implementations of two Semi-Global (Block) Matching (i.e. SGM or SGBM) algorithms:

* [NVIDIA VisionWorks](https://developer.nvidia.com/embedded/visionworks), and
* [Fixstars libSGM](https://github.com/fixstars/libSGM)

With stubs for an OpenCV-based nodelet in process.

This version is based on [whill-lab's](https://github.com/whill-labs) [upstream package](https://github.com/whill-labs/gpu_stereo_image_proc), but has diverged significantly in structure.

Each algorithm is in its own ROS package so individual packages can be disabled by adding them to the [Catkin blocklist](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_config.html#whitelisting-and-blacklisting-packages). 

As this package provides the baseline stereo capabilities on our [Trisect underwater trifocal sensor](https://trisect-perception-sensor.gitlab.io/), our primary development environment is the Jetson NX running Jetpack 4.4.x.   It also assumes the Trisect's customized [OpenCV](https://gitlab.com/apl-ocean-engineering/jetson/buildopencv) and [ROS](https://gitlab.com/apl-ocean-engineering/jetson/buildros1) builds.

## Installation

Pull this repo into a catkin workspace and build

```sh
cd <path-to-your-catkin-workspace>/src
git clone https://github.com/WHILL/gpu_stereo_image_proc.git
cd ..
catkin build
```

`gpu_stereo_image_proc_libsgm` will automatically pull in libSGM as an external project.


## Usage

### Nodelets

This package contains nodelets for creating disparity images from stereo:

* `gpu_stereo_image_proc_libsgm/libsgm_disparity`:  Nodelet which wraps Fixstars libSGM.
* `gpu_stereo_image_proc_visionworks/vx_disparity`:  Nodelet which wraps NVIDIA VisionWorks.

Basic usage of this package (e.g. subscribed/published topics, node structure) is compatible with [ros-perception/image_pipeline/stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/melodic/stereo_image_proc). Refer to its [wiki](http://wiki.ros.org/stereo_image_proc?distro=melodic) for quick start.

### Parameters

Most of the parameters can be configured via [dynamic reconfigure](http://wiki.ros.org/dynamic_reconfigure). These wiki pages describes configurable parameters.

- [Parameter (libSGM)](https://github.com/WHILL/gpu_stereo_image_proc/wiki/Parameter-(libSGM))
- [Parameter (VisionWorks)](https://github.com/WHILL/gpu_stereo_image_proc/wiki/Parameter-(VisionWorks))

## Licenses

This software maintain's the upstream package's [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause).

The underlying code of this package is forked from [ros-perception/image_pipeline/stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/melodic/stereo_image_proc) which is distibuted under the [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause).

This package includes [libSGM](https://github.com/fixstars/libSGM) which is distributed under the [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0).

This package depends on [VisionWorks](https://developer.nvidia.com/embedded/visionworks) which NVIDIA reserves all the copyrights of.
