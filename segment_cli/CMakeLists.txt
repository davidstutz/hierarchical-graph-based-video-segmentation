cmake_minimum_required(VERSION 2.8)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_C_FLAGS} -std=c++0x")

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
include_directories(../lib_segment/ ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(segment_cli main.cpp)
target_link_libraries(segment_cli segment ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})