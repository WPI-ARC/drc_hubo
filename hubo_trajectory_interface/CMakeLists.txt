cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Coverage)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Get Boost
rosbuild_add_boost_directories()

# Build and link the action interface
rosbuild_add_executable(hubo_trajectory_action src/hubo_trajectory_action.cpp)
rosbuild_link_boost(hubo_trajectory_action thread)

# Build and link the dummy backend for testing
rosbuild_add_executable(hubo_trajectory_dummy_interface src/hubo_trajectory_dummy_interface.cpp)
rosbuild_link_boost(hubo_trajectory_dummy_interface thread signals)

## Build and link the hubo-motion-rt trajectory interface
#rosbuild_add_executable(hubo_trajectory_interface src/hubo_trajectory_interface.cpp)
#target_link_libraries(hubo_trajectory_interface ach)
#rosbuild_link_boost(hubo_trajectory_interface thread signals)
