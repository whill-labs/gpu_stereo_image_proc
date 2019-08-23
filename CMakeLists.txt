cmake_minimum_required(VERSION 2.8)
project(gpu_stereo_image_proc)

find_package(catkin REQUIRED cv_bridge dynamic_reconfigure image_geometry image_proc image_transport message_filters nodelet sensor_msgs stereo_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++14 )
endif()

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/Disparity.cfg)

catkin_package(
  CATKIN_DEPENDS image_geometry image_proc sensor_msgs stereo_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include)

find_package(OpenCV REQUIRED) 
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
find_package(VisionWorks REQUIRED)
include_directories(${VisionWorks_INCLUDE_DIRS})
find_package(LibSGM REQUIRED)
include_directories(${LibSGM_INCLUDE_DIRS})


# See note in image_proc/CMakeLists.txt
add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)

# Nodelet library
add_library(${PROJECT_NAME} 
            src/libgpu_stereo_image_proc/vx_stereo_matcher.cpp
            src/libgpu_stereo_image_proc/vx_sgbm_processor.cpp 
            src/libgpu_stereo_image_proc/sgbm_processor.cpp 
            src/nodelets/disparity.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
                                      ${VisionWorks_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
add_library(libsgm-disparity
            src/libgpu_stereo_image_proc/libsgm_sgbm_processor.cpp 
            src/libgpu_stereo_image_proc/sgbm_processor.cpp 
            src/nodelets/libsgm_disparity.cpp)
target_link_libraries(libsgm-disparity ${catkin_LIBRARIES}
                                       ${OpenCV_LIBRARIES}
                                       ${LibSGM_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Standalone node
add_executable(stereoimageproc_exe src/nodes/stereo_image_proc.cpp)
target_link_libraries(stereoimageproc_exe ${PROJECT_NAME})
SET_TARGET_PROPERTIES(stereoimageproc_exe PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
install(TARGETS stereoimageproc_exe
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
