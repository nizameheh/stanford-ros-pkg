cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(wifi_sniffer src/wifi_sniffer.cpp)
target_link_libraries(wifi_sniffer iw boost_serialization)

rosbuild_add_executable(standalone src/standalone.cpp)
target_link_libraries(standalone wifi_sniffer)

rosbuild_add_executable(wifi_sniffer_node src/wifi_sniffer_node.cpp)
target_link_libraries(wifi_sniffer_node wifi_sniffer)

rosbuild_add_executable(mapper src/mapper.cpp)
target_link_libraries(mapper wifi_sniffer)

# these last two programs won't build... not sure why, haven't looked at them.
#rosbuild_add_executable(runner src/runner.cpp src/posePDF.cpp src/pose.cpp src/ConfigFileLoader.cpp src/Publisher.cpp)
#target_link_libraries(runner wifi_sniffer)

#rosbuild_add_executable(realRobotPosition src/realRobotPosition.cpp)
#target_link_libraries(realRobotPosition wifi_sniffer)
