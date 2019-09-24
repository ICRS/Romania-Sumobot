cmake_minimum_required(VERSION 2.8.3)
project(interfearence-launch)

find_package(catkin REQUIRED COMPONENTS
  interfearence-description
  interfearence-secret-weapon
  interfearence-hardware
  interfearence-localisation
)

catkin_package(
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)
