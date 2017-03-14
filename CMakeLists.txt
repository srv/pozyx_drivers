cmake_minimum_required(VERSION 2.8.3)

project(pozyx_drivers)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
 add_message_files(
  FILES
  AnchorInfo.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
 )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS message_runtime rospy std_msgs sensor_msgs geometry_msgs
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)