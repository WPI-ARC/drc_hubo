cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)
rosbuild_init()

#Find and use the LibXML++ library
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
find_package(LibXML++ REQUIRED)
include_directories(${LibXML++_INCLUDE_DIRS})
set(LIBS ${LIBS} ${LibXML++_LIBRARIES})

#Find and use the Eigen library
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#Set GCC to use the highest level of optimization
#set(CMAKE_CXX_FLAGS "-Ofast" ${CMAKE_CXX_FLAGS})

#common commands for building c++ executables and libraries
rosbuild_add_executable(xtf_test src/xtf_test.cpp src/xtf.cpp)
target_link_libraries(xtf_test ${LIBS})

rosbuild_add_executable(trajectory_evaluator_test src/main.cpp src/evaluator.cpp src/dtw.cpp src/xtf.cpp)
rosbuild_add_openmp_flags(trajectory_evaluator_test)
target_link_libraries(trajectory_evaluator_test rt ${LIBS})

rosbuild_add_executable(trajectory_evaluator src/node.cpp src/evaluator.cpp src/dtw.cpp src/xtf.cpp)
rosbuild_add_openmp_flags(trajectory_evaluator)
target_link_libraries(trajectory_evaluator rt ${LIBS})
