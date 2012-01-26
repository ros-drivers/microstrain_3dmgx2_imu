cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_library(LOG4CXX_LIBRARY log4cxx)

rosbuild_init()
rosbuild_gensrv()
rosbuild_add_executable(imu_node imu_node.cc)
target_link_libraries(imu_node 3dmgx2)
rosbuild_add_library(3dmgx2 3dmgx2.cc)

# Utility to recover ID from IMU
rosbuild_add_executable(get_id src/get_id.cpp)
target_link_libraries(get_id 3dmgx2)
target_link_libraries(get_id ${LOG4CXX_LIBRARY})
