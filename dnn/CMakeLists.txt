cmake_minimum_required(VERSION 2.8)

project(usingopencv)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
add_executable(main main.cpp cv-helpers.hpp)
target_link_libraries(main ${realsense2_LIBRARY} ${OpenCV_LIBS})

