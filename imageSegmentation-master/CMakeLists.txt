cmake_minimum_required(VERSION 2.8)

project(ImageSeg)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
add_executable(main main.cpp)
INCLUDE_DIRECTORIES(~/)

target_link_libraries(main ${OpenCV_LIBS} seg_shared ${realsense2_LIBRARY})
add_library(seg_shared SHARED histogram.cpp color.cpp texture.cpp)

set( CMAKE_BUILD_TYPE "Debug" )


