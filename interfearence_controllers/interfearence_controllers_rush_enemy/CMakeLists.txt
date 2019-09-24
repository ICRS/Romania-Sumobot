cmake_minimum_required(VERSION 2.8.3)
project(interfearence_controllers_rush_enemy)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy tf2_ros
)
