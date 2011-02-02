cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_library(ancient_powercube ancient_powercube.cpp)

rosbuild_add_executable(home home.cpp)
target_link_libraries(home ancient_powercube)

rosbuild_add_executable(node node.cpp)
target_link_libraries(node ancient_powercube)

rosbuild_add_executable(set_pan_tilt set_pan_tilt.cpp)
