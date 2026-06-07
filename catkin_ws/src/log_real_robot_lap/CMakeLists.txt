cmake_minimum_required(VERSION 3.0.2)
project(log_real_robot_lap)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  message_generation
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  OdomRecordReal.action
)

## Generate messages and services with dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs actionlib actionlib_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)