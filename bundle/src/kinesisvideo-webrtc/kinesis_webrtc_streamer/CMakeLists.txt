# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.10.3)
project(kinesis_webrtc_streamer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(aws_common REQUIRED)
find_package(aws_ros2_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kinesis_video_msgs REQUIRED)
find_package(kinesis_webrtc_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

###########
## Build ##
###########

## Declare source files
set(KINESIS_WEBRTC_STREAMER_SRC 
    src/streamer_params.cpp
    src/subscriber_callbacks.cpp
    src/webrtc_node.cpp
)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/main.cpp ${KINESIS_WEBRTC_STREAMER_SRC})
add_library(${PROJECT_NAME}_lib ${KINESIS_WEBRTC_STREAMER_SRC})

## Specify include directories
target_include_directories(${PROJECT_NAME} PRIVATE 
  include
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC 
  include
)

## Specify libraries to link a library or executable target against
ament_target_dependencies(${PROJECT_NAME}
  aws_common
  aws_ros2_common
  geometry_msgs
  kinesis_video_msgs
  kinesis_webrtc_manager
  rclcpp
  std_msgs
)

ament_target_dependencies(${PROJECT_NAME}_lib
  aws_common
  aws_ros2_common
  geometry_msgs
  kinesis_video_msgs
  kinesis_webrtc_manager
  rclcpp
  std_msgs
)

target_link_libraries(${PROJECT_NAME} kinesis_webrtc_manager)
target_link_libraries(${PROJECT_NAME}_lib kinesis_webrtc_manager)

#############
## Install ##
#############

# Install executables
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  # Test for subscriber callbacks
  ament_add_gmock(test_subscriber_callbacks
    test/subscriber_callbacks_test.cpp)
      
  target_include_directories(test_subscriber_callbacks
    PRIVATE include ${INCLUDE_DIRS})
  target_link_libraries(test_subscriber_callbacks
          ${PROJECT_NAME}_lib
          kinesis_webrtc_manager)
  
  # Test for streamer parameters
  ament_add_gmock(test_streamer_params
    test/streamer_params_test.cpp)

  target_include_directories(test_streamer_params
    PRIVATE include ${INCLUDE_DIRS})
  target_link_libraries(test_streamer_params
          ${PROJECT_NAME}_lib
          kinesis_webrtc_manager)

  # Test for WebRTC node
  ament_add_gmock(test_webrtc_node
    test/webrtc_node_test.cpp)
  
  target_include_directories(test_webrtc_node
    PRIVATE include ${INCLUDE_DIRS})
  target_link_libraries(test_webrtc_node
          ${PROJECT_NAME}_lib
          kinesis_webrtc_manager)
endif()
# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  aws_common
  aws_ros2_common
  geometry_msgs
  kinesis_video_msgs
  kinesis_webrtc_manager
  rclcpp
  std_msgs
)

ament_package()
