cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(binary2ascii)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (binary2ascii ./src/binary2ascii.cpp)
target_link_libraries (binary2ascii ${PCL_LIBRARIES})