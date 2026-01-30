# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(deepracer_gazebo_system_plugin)

# Set C++17 standard for ROS 2 compatibility
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(deepracer_msgs REQUIRED)

# Find Gazebo Harmonic dependencies
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)

# Include directories
include_directories(
  include
  ${gz-sim8_INCLUDE_DIRS}
  ${gz-plugin2_INCLUDE_DIRS}
)

# ROS 2 System Plugin
add_library(deepracer_gazebo_system_plugin SHARED
  src/deepracer_gazebo_system_plugin.cpp
)

# Set target properties
set_target_properties(deepracer_gazebo_system_plugin PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

# Link libraries
target_link_libraries(deepracer_gazebo_system_plugin
  ${gz-sim8_LIBRARIES}
  ${gz-plugin2_LIBRARIES}
)

# ROS 2 dependencies
ament_target_dependencies(deepracer_gazebo_system_plugin
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  deepracer_msgs
)

# Install plugin library
install(TARGETS deepracer_gazebo_system_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  
  # Unit tests
  ament_add_gtest(test_deepracer_plugin_unit
    test/unit/test_system_lifecycle.cpp
    test/unit/test_visual_services.cpp
    test/unit/test_visual_manipulation.cpp
    test/unit/test_model_link_states.cpp
    test/unit/test_physics_control.cpp
    test/unit/test_bulk_visual_operations.cpp
  )
  
  target_link_libraries(test_deepracer_plugin_unit
    deepracer_gazebo_system_plugin
    ${gz-sim8_LIBRARIES}
    ${gz-plugin2_LIBRARIES}
  )
  
  target_include_directories(test_deepracer_plugin_unit PRIVATE
    include
    ${gz-sim8_INCLUDE_DIRS}
    ${gz-plugin2_INCLUDE_DIRS}
  )
  
  ament_target_dependencies(test_deepracer_plugin_unit
    rclcpp
    std_msgs
    std_srvs
    geometry_msgs
    deepracer_msgs
  )
  
  set_target_properties(test_deepracer_plugin_unit PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
  )
endif()

ament_package()
