cmake_minimum_required(VERSION 2.8.3)
project(stitchtron9000)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    dynamic_reconfigure
    nodelet
    image_transport
    cv_bridge
    )

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include(Mosaicer.txt)
