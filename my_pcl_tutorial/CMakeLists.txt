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

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})

find_package (VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})

# ---[ Point Cloud Library - Transforms
rosbuild_add_boost_directories()
rosbuild_add_executable (voxel_filter src/example.cpp )
rosbuild_add_executable (extract_indices src/extract_indices.cpp )
rosbuild_add_executable (correspondence_grouping src/correspondence_grouping.cpp)
rosbuild_add_executable (color_growing src/color_growing.cpp)
rosbuild_add_executable (viewer src/viewer.cpp)
rosbuild_add_executable (smoothing src/smoothing.cpp)
target_link_libraries (correspondence_grouping vtkCommon vtkRendering -lboost_system) 
target_link_libraries (voxel_filter -lboost_system)
target_link_libraries (color_growing -lboost_system)
target_link_libraries (extract_indices -lboost_system)
target_link_libraries (viewer vtkCommon vtkRendering -lboost_system)
target_link_libraries (smoothing -lboost_system)
#rosbuild_add_executable(pcl src/example.cpp)

