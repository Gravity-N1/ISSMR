cmake_minimum_required(VERSION 2.8)

project(usingopencv)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
add_executable(usingopencv main.cpp)
target_link_libraries(usingopencv ${realsense2_LIBRARY} ${OpenCV_LIBS})

