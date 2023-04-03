cmake_minimum_required(VERSION 3.0.2)
project(bobi_fishbot_v4_interface)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/modules/")

add_compile_options(-std=c++11)

file(GLOB INTERFACE_SOURCES
    src/*.cpp
)
list(FILTER INTERFACE_SOURCES EXCLUDE REGEX "fishbot_v4_interface_node.cpp")

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    sensor_msgs
    bobi_msgs
)

find_package(Qt5 REQUIRED Core Test)
find_package(Qt5DBus REQUIRED)

if(${Qt5_FOUND})
  message(STATUS "Qt5 found")
else()
  message(STATUS "Qt5 not found")
endif()

if(${Qt5DBus_FOUND})
  message(STATUS "Qt5DBus found")
else()
  message(STATUS "Qt5DBus not found")
endif()

catkin_package(
  CATKIN_DEPENDS bobi_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

qt5_wrap_cpp(MOC_SOURCES include/bobi_fishbot_v4_interface/dbus_interface.h)

add_executable(fishbot_v4_interface_node src/fishbot_v4_interface_node.cpp ${MOC_SOURCES} ${INTERFACE_SOURCES})
target_link_libraries(fishbot_v4_interface_node ${catkin_LIBRARIES} Qt5::DBus Qt5::Test)
add_dependencies(fishbot_v4_interface_node ${catkin_EXPORTED_TARGETS} bobi_msgs_generate_messages_cpp)
