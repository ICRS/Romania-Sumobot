cmake_minimum_required(VERSION 2.8.3)
project(interfearence_controllers_train_rush_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_manager_msgs
  gazebo_msgs
  geometry_msgs
  interfearence_msgs
  nav_msgs
  rospy
  std_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    controller_manager_msgs 
    gazebo_msgs
    geometry_msgs
    interfearence_msgs
    nav_msgs
    rospy
    std_msgs
    tf2_ros
)
