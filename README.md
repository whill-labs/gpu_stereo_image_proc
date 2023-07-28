# gpu_stereo_image_proc

> NOTE:  As this repo has increasing focused on [Trisect underwater trifocal sensor](https://trisect-perception-sensor.gitlab.io/), I've renamed the primary branch to `trisect-dev` and retired the previous `melodic-devel` branch.

## Overview

This package provides the baseline stereo capabilities on our [Trisect underwater trifocal sensor](https://trisect-perception-sensor.gitlab.io/).  Given that focus, our primary development environment is the Jetson NX running Jetpack 4.4.x. with assumes the Trisect's customized [OpenCV and ROS1](https://gitlab.com/rsa-perception-sensor/trisect_environment) builds.

It includes ROS nodelets for CUDA implementations of the following Semi-Global (Block) Matching (i.e. SGM or SGBM) algorithms:

* [NVIDIA VisionWorks](https://developer.nvidia.com/embedded/visionworks)
* NVIDIA VPI

With stubs for a pure-OpenCV version in progress.  Each algorithm is in its own ROS package, compilation of individual packages can be disabled by adding them to the [Catkin skiplist](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_config.html#buildlisting-and-skiplisting-packages):

```
catkin config --skiplist gpu_stereo_image_proc_opencv
```

It also includes [Fixstars libSGM](https://github.com/fixstars/libSGM), however this ROS package is disabled by default.  It can be enabled by setting the CMake variable `BUILD_GPU_STEREO_IMAGE_PROC_LIBSGM`:

```
catkin config --cmake-args -DBUILD_GPU_STEREO_IMAGE_PROC_LIBSGM=True
```

In summary:

| Package | Current Status on Trisect |
|---------|---------------------------|
| gpu_stereo_image_proc_visionworks | Preferred version on Trisect.   Expect to be phased out in later version of Jetpack |
| gpu_stereo_image_proc_vpi | Almost complete on Trisect, with limited functionality due to early version of VPI.   Expect this to be main focus of development going forward. |
| gpu_stereo_image_proc_opencv | Not complete.|
| gpu_stereo_image_proc_libsgm | **Deprecated.** Can be manually enabled. |


## Installation

Pull this repo into a catkin workspace and build

```sh
cd <path-to-your-catkin-workspace>/src
git clone https://github.com/apl-ocean-engineering/gpu_stereo_image_proc.git
cd ..
catkin build
```

Note that since we are tuning for our Jetson-based Trisect platform, it assumes you have a fairly current, CUDA-enabled OpenCV and NVidia's Visionworks and VPI libraries.

## Licenses

This version is based on [whill-lab's](https://github.com/whill-labs) [upstream package](https://github.com/whill-labs/gpu_stereo_image_proc), but has diverged significantly in structure.

This software maintain's the upstream package's [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause).

The underlying code of this package is forked from [ros-perception/image_pipeline/stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/melodic/stereo_image_proc) which is distibuted under the [3-Clause BSD License](https://opensource.org/licenses/BSD-3-Clause).

This package includes [libSGM](https://github.com/fixstars/libSGM) which is distributed under the [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0).

This package depends on [VisionWorks](https://developer.nvidia.com/embedded/visionworks) which NVIDIA reserves all the copyrights of.
