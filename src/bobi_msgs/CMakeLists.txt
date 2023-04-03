cmake_minimum_required(VERSION 3.0.2)
project(bobi_msgs)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  RPY.msg
  Pose.msg
  PoseStamped.msg
  PoseVec.msg
  MotorVelocities.msg
  MotorVelocitiesStamped.msg
  ProximitySensors.msg
  DroppedMessages.msg
  Temperature.msg
  SpeedEstimateVec.msg
  KickSpecs.msg
  DLISpecs.msg
  NumAgents.msg
  TargetPose.msg
)

add_service_files(
  FILES
  EnableIR.srv
  ReturnVelocity.srv
  ReturnDroppedMsgs.srv
  MaxAcceleration.srv
  ConvertCoordinates.srv
  DutyCycle.srv
  ReturnTemperature.srv
  GetNumAgents.srv
  BacIntervene.srv
)

generate_messages(DEPENDENCIES
  std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
