cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(removeDynamic)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (removeDynamic removeDynamic.cpp)
target_link_libraries (removeDynamic ${PCL_LIBRARIES})
target_link_libraries (removeDynamic libyaml-cpp.so)
