cmake_minimum_required(VERSION 2.8.3)
project(stitchtron9000)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    dynamic_reconfigure
    nodelet
    )

include(Mosaicer.txt)
