cmake_minimum_required(VERSION 3.0.2)
project(bobi_vision)

add_compile_options(-std=c++17)

file(GLOB TRACKING_SOURCES
  src/*.cpp
)
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "individual_tracking_node.cpp")
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "robot_tracking_node.cpp")
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "synchronous_tracking_node.cpp")
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "visualization_node.cpp")
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "trajectory_identification_node.cpp")
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "aruco_tracking_node.cpp")
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "coordinate_converter_node.cpp")
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "click_to_waypoint_node.cpp")
list(FILTER TRACKING_SOURCES EXCLUDE REGEX "click_to_waypoint_rqt_node.cpp")

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge
  dynamic_reconfigure
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  config/IndividualTracker.cfg
  config/ColourDetector.cfg
  config/TrajectoryIdentificationConfig.cfg
)

catkin_package(
  CATKIN_DEPENDS bobi_msgs
  DEPENDS EIGEN3
  INCLUDE_DIRS include

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(individual_tracking_node src/individual_tracking_node.cpp ${TRACKING_SOURCES})
target_link_libraries(individual_tracking_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(individual_tracking_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)

add_executable(robot_tracking_node src/robot_tracking_node.cpp ${TRACKING_SOURCES})
target_link_libraries(robot_tracking_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(robot_tracking_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)

add_executable(synchronous_tracking_node src/synchronous_tracking_node.cpp ${TRACKING_SOURCES})
target_link_libraries(synchronous_tracking_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(synchronous_tracking_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)

add_executable(visualization_node src/visualization_node.cpp ${TRACKING_SOURCES})
target_link_libraries(visualization_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(visualization_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)

add_executable(trajectory_identification_node src/trajectory_identification_node.cpp ${TRACKING_SOURCES})
target_link_libraries(trajectory_identification_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(trajectory_identification_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)

add_executable(click_to_waypoint_node src/click_to_waypoint_node.cpp ${TRACKING_SOURCES})
target_link_libraries(click_to_waypoint_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(click_to_waypoint_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)

add_executable(click_to_waypoint_rqt_node src/click_to_waypoint_rqt_node.cpp ${TRACKING_SOURCES})
target_link_libraries(click_to_waypoint_rqt_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(click_to_waypoint_rqt_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)

add_executable(coordinate_converter_node src/coordinate_converter_node.cpp ${TRACKING_SOURCES})
target_link_libraries(coordinate_converter_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(coordinate_converter_node ${catkin_EXPORTED_TARGETS} bobi_msgs_generate_messages_cpp)

add_executable(aruco_tracking_node src/aruco_tracking_node.cpp ${TRACKING_SOURCES})
target_link_libraries(aruco_tracking_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(aruco_tracking_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg bobi_msgs_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)