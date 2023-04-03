cmake_minimum_required(VERSION 3.0.2)
project(bobi_lurebot_interface)

add_compile_options(-std=c++17)

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

add_subdirectory(deps/SimpleBLE ${CMAKE_BINARY_DIR}/simpleble)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SIMPLEBLE_INCLUDES}
  ${SIMPLEBLUEZ_INCLUDES}
)

add_executable(lurebot_interface_node src/lurebot_interface_node.cpp)
target_link_libraries(lurebot_interface_node ${catkin_LIBRARIES} simpleble)
add_dependencies(lurebot_interface_node ${catkin_EXPORTED_TARGETS}  bobi_msgs_generate_messages_cpp)

add_executable(lurebot_dummy_interface_node src/lurebot_dummy_interface_node.cpp)
target_link_libraries(lurebot_dummy_interface_node ${catkin_LIBRARIES})
add_dependencies(lurebot_dummy_interface_node ${catkin_EXPORTED_TARGETS}  bobi_msgs_generate_messages_cpp)

add_executable(timed_motor_vel src/test/timed_motor_vel.cpp)
target_link_libraries(timed_motor_vel ${catkin_LIBRARIES})
add_dependencies(timed_motor_vel ${catkin_EXPORTED_TARGETS}  bobi_msgs_generate_messages_cpp)