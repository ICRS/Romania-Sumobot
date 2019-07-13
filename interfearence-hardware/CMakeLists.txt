cmake_minimum_required(VERSION 2.8.3)
project(interfearence-hardware)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  hardware_interface
  controller_manager
  geometry_msgs
  std_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hardware
  src/hardware_controller.cpp
  src/hardware/interfearence.cpp)

add_dependencies(hardware ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hardware
  ${catkin_LIBRARIES}
)

install(TARGETS hardware
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/velocity_control.py
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
