cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (LidarProj ClusterEngine/ClusterEngine.cpp processPointClouds/processPointClouds.cpp render/render.cpp SegmentEngine/SegmentEngine.cpp LidarProj.cpp)
#add_executable (LidarProj render/render.cpp)
target_link_libraries (LidarProj ${PCL_LIBRARIES})




