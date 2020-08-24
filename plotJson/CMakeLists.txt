cmake_minimum_required(VERSION 2.8.3)
project(plotJson)

add_definitions(-std=c++11)

find_package(PCL 1.8 REQUIRED)
# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# Json
set(Jsoncpp_INCLUDE_PATH "/usr/include/jsoncpp")
set(Jsoncpp_LIBRARY "/usr/lib/x86_64-linux-gnu/libjsoncpp.so")
include_directories(${Jsoncpp_INCLUDE_PATH})
# own
include_directories(include)
add_executable(plotJson src/plotJson.cpp)
target_link_libraries(plotJson ${Jsoncpp_LIBRARY})
target_link_libraries(plotJson ${PCL_LIBRARIES})