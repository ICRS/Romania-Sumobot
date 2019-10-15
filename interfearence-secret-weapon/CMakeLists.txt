cmake_minimum_required(VERSION 2.8.3)
project(interfearence-secret-weapon)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  sensor_msgs
  nav_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(lidar_processor 
    src/lidar_processor.cpp
    src/lidar_processor/robot_finder.cpp
    src/dbscan/dbscan.cpp
)

add_dependencies(lidar_processor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(lidar_processor
  Eigen3::Eigen
  ${catkin_LIBRARIES}
)

install(TARGETS lidar_processor
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
