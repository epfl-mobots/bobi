cmake_minimum_required(VERSION 3.0.2)
project(bobi_thymio_interface)

add_compile_options(-std=c++11)


find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    sensor_msgs
    bobi_msgs
)

catkin_package(
  CATKIN_DEPENDS bobi_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


