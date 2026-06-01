cmake_minimum_required(VERSION 3.0.2)
project(log_robot_lap)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  geometry_msgs
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(
  FILES
  OdomRecord.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    actionlib
    actionlib_msgs
    geometry_msgs
    std_msgs
    message_runtime
)	
