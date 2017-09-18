project(narf)

cmake_minimum_required(VERSION 2.8)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(narf main.cpp)

add_executable(generateRangeImg learnRangImg.cpp)

target_link_libraries(narf ${PCL_LIBRARIES})
target_link_libraries(generateRangeImg ${PCL_LIBRARIES})

#install(TARGETS narf RUNTIME DESTINATION bin)
