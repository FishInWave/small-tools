cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cvclose)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable (cvclose cvclose.cpp)
target_link_libraries (cvclose ${OpenCV_LIBS})
MESSAGE(STATUS "OpenCV libs:${OpenCV_LIBS}")