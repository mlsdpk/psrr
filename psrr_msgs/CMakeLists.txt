cmake_minimum_required(VERSION 2.8.3)
project(psrr_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  Path.msg
  State.msg
)

add_service_files(
  FILES
  FootPrint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs message_runtime
)