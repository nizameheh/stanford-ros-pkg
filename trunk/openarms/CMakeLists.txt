cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#find_package(SDL REQUIRED)
rosbuild_init(openarms)
rosbuild_genmsg()

#uncomment if you have defined messages
#genmsg()
#uncomment if you have defined services
#gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} filters/ekf.cpp)
#include_directories(${SDL_INCLUDE_DIR})
#include_directories(filters)
#rosbuild_add_executable(unpowered drivers/unpowered.cpp)
#target_link_libraries(unpowered ${SDL_LIBRARY} GL GLU)

#target_link_libraries(viewer ${SDL_LIBRARY} GL GLU)

#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_find_package(gazebo_robot_description)

#file(GLOB openarms_stl_files ${CMAKE_CURRENT_SOURCE_DIR}/meshes/*.stl)
#set(openarms_gen_files "")
#set(openarms_out_path ${gazebo_robot_description_PACKAGE_PATH}/Media/models/other)
#MAKE_DIRECTORY(${openarms_out_path})
#foreach(it ${openarms_stl_files})
#  get_filename_component(basename ${it} NAME_WE)
  #  message("hello ", ${it})

  # convert to ogre files
#  add_custom_command(
#    OUTPUT ${openarms_out_path}/${basename}.mesh
#    COMMAND rosrun 
#    ARGS ogre_tools stl_to_mesh ${it} ${openarms_out_path}/${basename}.mesh
#    DEPENDS ${it})

#  set(openarms_gen_files ${openarms_gen_files} ${openarms_out_path}/${basename}.mesh)
#endforeach(it)

#add_custom_target(media_files ALL DEPENDS ${openarms_gen_files})

#rosbuild_add_executable(raw_gyros_currents drivers/raw_gyros_currents.cpp)
#rosbuild_add_executable(raw_accels drivers/raw_accels.cpp)

#rosbuild_add_executable(ekf filters/ekf.cpp)

#rosbuild_add_executable(pid control/pid.cpp)
#rosbuild_add_executable(vel-bounce control/vel-bounce.cpp)

#rosbuild_add_executable(openarms-usb drivers/openarms-usb.cpp)

#rosbuild_add_executable(gripper_wrist_hack drivers/gripper_wrist_hack.cpp)

#rosbuild_add_executable(mc_hack drivers/mc_hack.cpp)

#rosbuild_add_executable(minimal_mc drivers/minimal_mc.cpp)
#rosbuild_add_executable(dual_minimal_mc drivers/dual_minimal_mc.cpp)
#rosbuild_add_executable(oa-minimal drivers/oa-minimal.cpp)
#rosbuild_add_executable(oa2-minimal drivers/oa2-minimal.cpp)

#rosbuild_add_executable(oa-hand drivers/oa-hand.cpp)
rosbuild_add_executable(oa-gripper drivers/oa-gripper.cpp)
rosbuild_add_executable(armbus drivers/armbus.cpp)
rosbuild_add_executable(oa-torso drivers/oa-torso.cpp)
rosbuild_add_executable(oa-upperarm drivers/oa-upperarm.cpp)
rosbuild_add_executable(oa-state state/oa-state.cpp)
rosbuild_add_executable(wiimote_master master/wiimote_master.cpp)
#rosbuild_add_executable(laser-tilter drivers/laser-tilter.cpp)
#rosbuild_add_executable(wrist-torques drivers/wrist-torques.cpp)

