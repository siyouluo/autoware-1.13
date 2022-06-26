cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(points_map_process)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (points_map_process src/points_map_process.cpp)
target_link_libraries (points_map_process ${PCL_LIBRARIES})
