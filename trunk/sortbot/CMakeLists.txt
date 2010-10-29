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
rosbuild_init(sortbot)
#rosbuild_genmsg()
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} filters/ekf.cpp)
#include_directories(${SDL_INCLUDE_DIR})
#include_directories(filters)
#rosbuild_add_executable(unpowered drivers/unpowered.cpp)
#target_link_libraries(unpowered ${SDL_LIBRARY} GL GLU)

