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
rosbuild_find_ros_package(build_tools)
include(${build_tools_PACKAGE_PATH}/scripts/build_common.cmake)

#uncomment if you have defined messages
rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#common commands for building c++ executables and libraries

rosbuild_add_boost_directories()

rosbuild_add_library(${PROJECT_NAME} 
  src/novatel_gps.cpp
	src/novatel_message_parser.cpp)
rosbuild_link_boost(${PROJECT_NAME} system)

rosbuild_add_library(${PROJECT_NAME}_nodelets
    src/nodelets/novatel_gps_nodelet.cpp
)
target_link_libraries(${PROJECT_NAME}_nodelets ${PROJECT_NAME})

rosbuild_add_executable(novatel_gps_node src/nodes/novatel_gps_node.cpp)
