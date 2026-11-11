# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(deepracer_msgs)


# Default to C++20 (the floor for ROS 2 Lyrical)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Message files
  "msg/ModelState.msg"
  "msg/LinkState.msg"
  # Service files
  "srv/GetLightNames.srv"
  "srv/GetVisualNames.srv"
  "srv/GetVisual.srv"
  "srv/GetVisuals.srv"
  "srv/GetLinkStates.srv"
  "srv/GetModelStates.srv"
  "srv/GetModelProperties.srv"
  "srv/SetVisualColor.srv"
  "srv/SetVisualColors.srv"
  "srv/SetVisualTransparency.srv"
  "srv/SetVisualTransparencies.srv"
  "srv/SetVisualVisible.srv"
  "srv/SetVisualVisibles.srv"
  "srv/SetVisualPose.srv"
  "srv/SetVisualPoses.srv"
  "srv/SetVisualMesh.srv"
  "srv/SetVisualMeshes.srv"
  "srv/SetLinkStates.srv"
  "srv/SetModelStates.srv"
  "srv/SetLightProperties.srv"
  "srv/SpawnModel.srv"
  "srv/DeleteModel.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
