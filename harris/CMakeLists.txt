project(harris)
cmake_minimum_required(VERSION 3.2)

find_package( OpenCV REQUIRED )
#opencv2.8以后的版本都没有必要再包含头文件目录进来
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(harris main.cpp)
add_library(geometrydescriptor geometrydescriptor.cpp)

target_link_libraries(harris ${OpenCV_LIBS} ${PCL_LIBRARIES})

#install(TARGETS harris RUNTIME DESTINATION bin)
