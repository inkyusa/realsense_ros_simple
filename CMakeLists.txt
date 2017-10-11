cmake_minimum_required(VERSION 2.8.0)
project(realsense_ros_simple)

add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(realsense_ros_simple
    src/lib/zr300.cpp
    src/lib/helper.cpp
    src/lib/time_sync.cpp
)

target_link_libraries(realsense_ros_simple realsense)

cs_add_executable(zr300node src/nodes/zr300node.cpp)
target_link_libraries(zr300node realsense_ros_simple)

cs_install()
cs_export()
