cmake_minimum_required(VERSION 2.8.3)
project(interfearence-master)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy
)

#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#)

#add_executable(master 
#  src/master.cpp
#)

#add_dependencies(master ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(master
#  ${catkin_LIBRARIES}
#)

#install(TARGETS master
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
