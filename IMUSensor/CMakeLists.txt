cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

include_directories ("${PROJECT_SOURCE_DIR}/msg_gen/cpp/include/${PROJECT_NAME}")

include_directories ("${PROJECT_SOURCE_DIR}/CmtSrc")

rosbuild_add_library(mtig CmtSrc/xsens_time.cpp CmtSrc/xsens_std.cpp CmtSrc/cmt1.cpp CmtSrc/cmt2.cpp CmtSrc/cmt3.cpp CmtSrc/cmtscan.cpp CmtSrc/cmtmessage.cpp CmtSrc/cmtpacket.cpp)

rosbuild_add_library(imucap src/FPSPrinter.cpp src/IMU.cpp)

rosbuild_add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher imucap mtig)
