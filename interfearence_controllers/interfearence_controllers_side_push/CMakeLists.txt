cmake_minimum_required(VERSION 2.8.3)
project(interfearence_controllers_side_push)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy 
)
