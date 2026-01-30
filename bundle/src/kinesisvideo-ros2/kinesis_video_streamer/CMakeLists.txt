#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

cmake_minimum_required(VERSION 3.20)
project(kinesis_video_streamer)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(kinesis_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aws_ros2_common REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(aws_common REQUIRED)
find_package(image_transport REQUIRED)
find_package(kinesis_video_msgs REQUIRED)
find_package(rmw_implementation REQUIRED)

include_directories(include)

set(PRODUCER_LIBRARY producer)
set(LOG_LIBRARY log4cplus)
set(INCLUDE_DIRS ${kinesis_manager_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}
    ${aws_ros2_common_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}
    ${aws_common_INCLUDE_DIRS} ${image_transport_INCLUDE_DIRS}
    ${kinesis_video_msgs_INCLUDE_DIRS}
    ${rmw_implementation_INCLUDE_DIRS})
set(LIB_DEPS ${kinesis_manager_LIBRARIES} ${rclcpp_LIBRARIES}
    ${aws_ros2_common_LIBRARIES} ${ament_cmake_LIBRARIES} ${aws_common_LIBRARIES}
    ${image_transport_LIBRARIES} ${PRODUCER_LIBRARY}
    ${kinesis_video_msgs_LIBRARIES} ${rmw_implementation_LIBRARIES} ${LOG_LIBRARY})

###########
## Build ##
###########
## Declare a C++ executable
set(KINESIS_VIDEO_STREAMER_SRC src/ros_stream_subscription_installer.cpp
  src/subscriber_callbacks.cpp src/streamer.cpp src/main.cpp)
add_executable(${PROJECT_NAME} ${KINESIS_VIDEO_STREAMER_SRC})
add_library(${PROJECT_NAME}_lib ${KINESIS_VIDEO_STREAMER_SRC})

## Specify include directories
target_include_directories(${PROJECT_NAME} PRIVATE include ${INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_lib PUBLIC include ${INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${LIB_DEPS} image_transport::image_transport)
target_link_libraries(${PROJECT_NAME}_lib ${LIB_DEPS} image_transport::image_transport)

ament_target_dependencies(${PROJECT_NAME} image_transport)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY
  DESTINATION lib/${PROJECT_NAME}
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_export_dependencies(h264_encoder_core)
ament_export_dependencies(aws_common)
ament_export_dependencies(aws_ros2_common)
ament_export_dependencies(image_transport)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(kinesis_video_msgs)
ament_export_dependencies(ament_cmake_gtest)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rmw_implementation)


#############
## Tests ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  
  ament_add_gtest(test_kinesis_video_streamer
    test/kinesis_video_streamer_test.cpp)
  target_include_directories(test_kinesis_video_streamer
    PRIVATE include ${KPL_INCLUDES_PATH}
    ${kinesis_manager_INCLUDE_DIRS})
  target_link_libraries(test_kinesis_video_streamer
          ${PROJECT_NAME}_lib
          ${LIB_DEPS})

  ament_add_gmock(test_streamer_node
    test/streamer_node_test.cpp)
  target_include_directories(test_streamer_node
    PRIVATE include ${KPL_INCLUDES_PATH}
    ${kinesis_manager_INCLUDE_DIRS})
  target_link_libraries(test_streamer_node
          ${PROJECT_NAME}_lib
          ${LIB_DEPS})

  ament_add_gmock(test_subscriber_callbacks
    test/subscriber_callbacks_test.cpp)
  target_include_directories(test_subscriber_callbacks
    PRIVATE include ${KPL_INCLUDES_PATH}
    ${kinesis_manager_INCLUDE_DIRS})
  target_link_libraries(test_subscriber_callbacks
          ${PROJECT_NAME}_lib
          ${LIB_DEPS})
endif()

ament_package()
