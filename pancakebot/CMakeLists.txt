cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#find_package(SDL REQUIRED)
rosbuild_init(pancakebot)
rosbuild_genmsg()
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} filters/ekf.cpp)
include_directories(include)
rosbuild_add_library(pancakes src/pancakes.cpp)

rosbuild_add_executable(pancake_detector_node src/pancake_detector_node.cpp)
target_link_libraries(pancake_detector_node pancakes)
