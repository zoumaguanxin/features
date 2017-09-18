project(feature2d)

cmake_minimum_required(VERSION 3.2)

find_package( OpenCV REQUIRED )
#opencv2.8以后的版本都没有必要再包含头文件目录进来
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

add_executable(feature2d main.cpp)

target_link_libraries(feature2d ${OpenCV_LIBS} ${PCL_LIBRARIES})

#install(TARGETS feature2d RUNTIME DESTINATION bin)
