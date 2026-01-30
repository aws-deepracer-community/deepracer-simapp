# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20)
project(h264_video_encoder)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)


find_package(h264_encoder_core REQUIRED)
find_package(aws_common REQUIRED)
find_package(aws_ros2_common REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kinesis_video_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rmw_implementation REQUIRED)

include_directories(include)


#############
## Compile ##
#############

set(h264_video_encoder_SRCS
  src/h264_video_encoder.cpp
)
set(h264_video_encoder_INCS
  include
  ${h264_encoder_core_INCLUDE_DIRS}
  ${aws_common_INCLUDE_DIRS}
  ${aws_ros2_common_INCLUDE_DIRS}
  ${kinesis_video_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS}
  ${rmw_implementation_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
)
set(h264_video_encoder_LIBS
  ${h264_encoder_core_LIBRARIES}
  ${kinesis_video_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${ament_cmake_LIBRARIES}
  ${aws_ros2_common_LIBRARIES}
  ${rmw_implementation_LIBRARIES}
  ${aws_common_LIBRARIES}
  ${image_transport_LIBRARIES}
)

# add the publisher example
add_executable(${PROJECT_NAME} src/main.cpp ${h264_video_encoder_SRCS})
add_library(${PROJECT_NAME}_lib STATIC ${h264_video_encoder_SRCS})

target_include_directories(${PROJECT_NAME} PRIVATE ${h264_video_encoder_INCS})
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${h264_video_encoder_INCS})

target_link_libraries(${PROJECT_NAME} ${h264_video_encoder_LIBS} image_transport::image_transport)
target_link_libraries(${PROJECT_NAME}_lib ${h264_video_encoder_LIBS} image_transport::image_transport)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Also install to libexec for backward compatibility with ROS launch systems
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION libexec/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)


#############
## Tests ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ## Add gtest based cpp test target and link libraries
  ament_add_gtest(test_h264_video_encoder
    test/h264_video_encoder_test.cpp
  )
  target_include_directories(test_h264_video_encoder
  PRIVATE include ${h264_video_encoder_INCS})
  target_link_libraries(test_h264_video_encoder
    ${PROJECT_NAME}_lib
    ${h264_video_encoder_LIBS}
    ${GMOCK_BOTH_LIBRARIES}
  )
endif()

ament_export_dependencies(h264_encoder_core)
ament_export_dependencies(aws_common)
ament_export_dependencies(aws_ros2_common)
ament_export_dependencies(image_transport)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(kinesis_video_msgs)
ament_export_dependencies(ament_cmake_gtest)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rmw_implementation)

ament_package()
