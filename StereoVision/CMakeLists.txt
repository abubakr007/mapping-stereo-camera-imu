cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

link_directories("${PROJECT_SOURCE_DIR}/lib")

include_directories("${PROJECT_SOURCE_DIR}/msg_gen/cpp/include/${PROJECT_NAME}")

rosbuild_add_library(stvision src/StereoAlgorithm.cpp src/DisparityMap.cpp src/PointCloudBuilder.cpp)

rosbuild_add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher stvision)

rosbuild_add_executable(publisher_imu src/publisher_imu.cpp)
target_link_libraries(publisher_imu stvision)

rosbuild_add_executable(viewer src/viewer.cpp)
target_link_libraries(viewer stvision)
