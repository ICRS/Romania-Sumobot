cmake_minimum_required(VERSION 3.10.2)
project(interfearence_hardware)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  hardware_interface
  controller_manager
  geometry_msgs
  std_msgs
)

find_package(PythonInterp 3.6 REQUIRED)
find_package(PythonLibs 3.6 REQUIRED)

catkin_package(
)

catkin_python_setup()

include_directories(
  include
  ${PYTHON_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
)

add_executable(hardware
  src/hardware_controller.cpp
  src/interfearence_hardware/interfearence.cpp)

add_dependencies(hardware ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hardware
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS hardware
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/velocity_control.py
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
