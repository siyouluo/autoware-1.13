cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(points_map_process)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(./include)

add_executable (transform_pointcloud src/transform_pointcloud.cpp)
target_link_libraries (transform_pointcloud ${PCL_LIBRARIES})


add_executable (split_pointcloud src/split_pointcloud.cpp)
target_link_libraries (split_pointcloud ${PCL_LIBRARIES})

