cmake_minimum_required(VERSION 2.8)

project(lfcornerextrator)

find_package(OpenCV REQUIRED)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

add_executable(lfcornerextrator main.cpp)

target_link_libraries(lfcornerextrator ${OpenCV_LIBS} ${PCL_LIBRARIES})

#install(TARGETS lfcornerextrator RUNTIME DESTINATION bin)
