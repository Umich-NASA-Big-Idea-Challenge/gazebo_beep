cmake_minimum_required(VERSION 3.10)
project(motor_plugin)

# Find the correct package for Gazebo Sim Fortress
find_package(ignition-gazebo6 REQUIRED)

# Include necessary directories
include_directories(${ignition-gazebo6_INCLUDE_DIRS})
link_directories(${ignition-gazebo6_LIBRARY_DIRS})

# Add the plugin source file
add_library(theory_motor_plugin SHARED src/theory_motor_plugin.cpp)

# Link the correct Gazebo Sim library
target_link_libraries(theory_motor_plugin ignition-gazebo6::ignition-gazebo6)

# Add the plugin source file
add_library(data_motor_plugin SHARED src/data_motor_plugin.cpp)

# Link the correct Gazebo Sim library
target_link_libraries(data_motor_plugin ignition-gazebo6::ignition-gazebo6)
