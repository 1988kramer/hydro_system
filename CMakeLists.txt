cmake_minimum_required(VERSION 2.8.3)
project(hydro_system)

find_package(catkin
  REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

add_message_files(
  FILES
  PhMsg.msg
  TempMsg.msg
  MotorHatCmd.msg
)

add_service_files(
  FILES
  CalibratePh.srv
  ChangeSetPoint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
)
