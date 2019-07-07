cmake_minimum_required(VERSION 2.8.3)
project(interfearence-localisation)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometry 
  src/odometry.cpp)

add_dependencies(odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(odometry
  ${catkin_LIBRARIES}
)

# Mark executables and/or libraries for installation
install(TARGETS odometry
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
