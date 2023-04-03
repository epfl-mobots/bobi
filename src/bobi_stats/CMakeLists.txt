cmake_minimum_required(VERSION 3.0.2)
project(bobi_stats)

add_compile_options(-std=c++17)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    sensor_msgs
    bobi_msgs
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  config/Logger.cfg
)

catkin_package(
  CATKIN_DEPENDS bobi_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
