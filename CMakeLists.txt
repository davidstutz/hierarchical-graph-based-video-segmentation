cmake_minimum_required(VERSION 2.8)

project(gbh_video_segmentation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O4 -g -std=c++0x")

add_subdirectory(lib_segment)
add_subdirectory(segment_cli)
add_subdirectory(optical_flow_cli)
