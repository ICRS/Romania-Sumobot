cmake_minimum_required(VERSION 2.8.3)
project(interfearence_openai)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  openai_ros
)

catkin_python_setup()

catkin_package(DEPENDS 
  rospy 
  openai_ros
)
