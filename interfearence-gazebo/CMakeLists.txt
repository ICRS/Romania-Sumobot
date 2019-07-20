cmake_minimum_required(VERSION 2.8.3)
project(interfearence-gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
)

catkin_package(
)
