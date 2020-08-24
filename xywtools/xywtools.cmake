include_directories(../xywtools/include/)
# json
set(Jsoncpp_INCLUDE_PATH "/usr/include/jsoncpp")
set(Jsoncpp_LIBRARY "/usr/lib/x86_64-linux-gnu/libjsoncpp.so")
include_directories(${Jsoncpp_INCLUDE_PATH})

#使用时必须target_link_libraries (analysJson ${Jsoncpp_LIBRARY})

