cmake_minimum_required(VERSION 3.1)
project(gpu_stereo_image_proc)

find_package(catkin REQUIRED cv_bridge dynamic_reconfigure image_geometry image_proc image_transport message_filters nodelet sensor_msgs stereo_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  #add_compile_options(-std=c++14 )

  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
  cfg/VXSGBM.cfg
  cfg/LIBSGM.cfg
  cfg/DisparityWLSFilter.cfg)

catkin_package(
  CATKIN_DEPENDS image_geometry image_proc sensor_msgs stereo_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include)

find_package(OpenCV 4 REQUIRED) 
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
find_package(VisionWorks REQUIRED)
include_directories(${VisionWorks_INCLUDE_DIRS})

find_package(CUDA REQUIRED)
set(CUDA_ARCH "-arch=sm_72" CACHE STRING "Value of the NVCC -arch option.")
if(${CMAKE_VERSION} VERSION_EQUAL 3.7 OR ${CMAKE_VERSION} VERSION_GREATER 3.7)
  option(AUTO_DETECT_ARCH "Detect local GPU compute arch automatically" ON)
endif()

if(DEFINED AUTO_DETECT_ARCH AND "${AUTO_DETECT_ARCH}")
	CUDA_SELECT_NVCC_ARCH_FLAGS(ARCH_FLAGS "Auto")
	set(CUDA_ARCH "${ARCH_FLAGS}" CACHE STRING "Value of the NVCC -arch option." FORCE)
endif()

include(ExternalProject)
ExternalProject_Add(
    ext-libSGM
    GIT_REPOSITORY "https://github.com/fixstars/libSGM.git"
    GIT_TAG     "master"
    PREFIX      "${CMAKE_CURRENT_BINARY_DIR}/ext/libSGM"
    INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/libSGM"
    SOURCE_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/libSGM"
    CMAKE_ARGS  "-DBUILD_OPENCV_WRAPPER=ON"
                "-DLIBSGM_SHARED=ON"
                "-DCUDA_ARCH=${CUDA_ARCH}"
                "-DAUTO_DETECT_ARCH=${AUTO_DETECT_ARCH}"
                "-DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/libSGM"
    BUILD_ALWAYS ON
)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/libSGM/include)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/libSGM/lib)

# See note in image_proc/CMakeLists.txt
add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)

# Nodelet library
set(VX_NODELET_NAME vx_stereo_image_proc)
add_library(${VX_NODELET_NAME}
            src/libgpu_stereo_image_proc/vx_stereo_matcher_base.cpp
            src/libgpu_stereo_image_proc/vx_stereo_matcher.cpp
            src/libgpu_stereo_image_proc/vx_bidirectional_stereo_matcher.cpp
            src/libgpu_stereo_image_proc/vx_conversions.cpp
            src/libgpu_stereo_image_proc/msg_conversions.cpp
            src/libgpu_stereo_image_proc/camera_info_conversions.cpp
            src/nodelets/vx_disparity.cpp)
target_link_libraries(${VX_NODELET_NAME} ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
                                      ${VisionWorks_LIBRARIES}
)
add_dependencies(${VX_NODELET_NAME} ${PROJECT_NAME}_gencfg)
install(TARGETS ${VX_NODELET_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

set(LIBSGM_NODELET_NAME libsgm_stereo_image_proc)
add_library(${LIBSGM_NODELET_NAME}
            src/libgpu_stereo_image_proc/libsgm_sgbm_processor.cpp 
            src/libgpu_stereo_image_proc/sgbm_processor.cpp 
            src/nodelets/libsgm_disparity.cpp)
target_link_libraries(${LIBSGM_NODELET_NAME} ${catkin_LIBRARIES}
                                             ${OpenCV_LIBRARIES}
                                             sgm
)

add_dependencies(${LIBSGM_NODELET_NAME} ext-libSGM ${PROJECT_NAME}_gencfg)
install(TARGETS ${LIBSGM_NODELET_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Standalone node
add_executable(vx_stereoimageproc_exe src/nodes/vx_stereo_image_proc.cpp)
target_link_libraries(vx_stereoimageproc_exe ${VX_NODELET_NAME})
SET_TARGET_PROPERTIES(vx_stereoimageproc_exe PROPERTIES OUTPUT_NAME vx_${PROJECT_NAME})
install(TARGETS vx_stereoimageproc_exe
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install the launch file
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

# install the include directory
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
