cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
include_directories(../lib_io/ ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(segment graph_segmentation.cpp evaluation.cpp io.cpp io_util.cpp)
target_link_libraries(segment ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})