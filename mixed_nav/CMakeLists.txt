cmake_minimum_required(VERSION 3.0.2)
project(mixed_nav)

find_package(catkin REQUIRED COMPONENTS
  move_base
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  actionlib_msgs
  actionlib
  message_generation
)

add_service_files(
  FILES
  AdjustPcdZ.srv
  Switch2DMap.srv
  SwitchPath.srv
  SwitchFloor.srv
  StartRecord.srv
)
add_action_files(
  FILES
  PathNavigation.action
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mixed_nav
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs actionlib_msgs visualization_msgs
 DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
