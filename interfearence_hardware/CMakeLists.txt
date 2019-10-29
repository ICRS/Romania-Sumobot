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

# Link to pigpio if we're on the pi
if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "armv7l")
    message("===============\ncompiling on arm\n===============")
    # Find the path to the pigpio includes.
    find_path(pigpio_INCLUDE_DIR
        NAMES pigpio.h pigpiod_if.h pigpiod_if2.h
        HINTS /usr/local/include)

    # Find the pigpio libraries.
    find_library(pigpio_LIBRARY
        NAMES libpigpio.so
        HINTS /usr/local/lib)
    find_library(pigpiod_if_LIBRARY
        NAMES libpigpiod_if.so
        HINTS /usr/local/lib)
    find_library(pigpiod_if2_LIBRARY
        NAMES libpigpiod_if2.so
        HINTS /usr/local/lib)

    # Set the pigpio variables to plural form to make them accessible for
    # the paramount cmake modules.
    set(pigpio_INCLUDE_DIRS ${pigpio_INCLUDE_DIR})
    set(pigpio_INCLUDES     ${pigpio_INCLUDE_DIR})

    # Handle REQUIRED, QUIET, and version arguments
    # and set the <packagename>_FOUND variable.
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(pigpio
        DEFAULT_MSG
        pigpio_INCLUDE_DIR pigpio_LIBRARY pigpiod_if_LIBRARY pigpiod_if2_LIBRARY)
else()
    message("====================\nSystem Processor:\n====================")
    message(${CMAKE_SYSTEM_PROCESSOR})
endif()

find_package(PythonInterp 3.6 REQUIRED)
find_package(PythonLibs 3.6 REQUIRED)

catkin_package(
)

catkin_python_setup()

include_directories(
  include
  ${PYTHON_INCLUDE_PATH}
  ${pigpio_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(hardware
  src/hardware_controller.cpp
  src/interfearence_hardware/interfearence.cpp)

add_dependencies(hardware ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hardware
  ${PYTHON_LIBRARIES}
  ${pigpio_LIBRARY}
  ${catkin_LIBRARIES}
)

install(TARGETS hardware
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/velocity_control.py
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
