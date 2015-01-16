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
    message_generation
    std_msgs
    )

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(src/extractor)
include(Mosaicer.txt)
