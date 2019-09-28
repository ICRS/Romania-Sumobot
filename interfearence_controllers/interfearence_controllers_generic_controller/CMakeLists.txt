cmake_minimum_required(VERSION 2.8.3)
project(interfearence_controllers_generic_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp std_msgs
)

catkin_python_setup()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(interfearence_controllers_generic_controller
  src/interfearence_controllers_generic_controller/generic_controller.cpp)

target_link_libraries(interfearence_controllers_generic_controller
  ${catkin_LIBRARIES}
)
