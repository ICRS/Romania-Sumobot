cmake_minimum_required(VERSION 2.8.3)
project(interfearence-gazebo)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  interfearence_msgs
  roscpp
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS plugins
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp interfearence_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(plugins)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(arena_edge_detection_plugin 
  plugins/arena_edge_detection_plugin.cpp
)
target_link_libraries(arena_edge_detection_plugin
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
  CameraPlugin
)

add_library(magnet_plugin
  plugins/magnet_plugin.cpp
)
target_link_libraries(magnet_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
