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
project(kinesis_video_msgs)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(diagnostic_msgs REQUIRED)

set(INCLUDE_DIRS
  ${ament_cmake_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS}
  ${diagnostic_msgs_INCLUDE_DIRS}
)
include_directories(${INCLUDE_DIRS})

## Generate messages in the 'msg' folder
## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KinesisVideoFrame.msg"
  "msg/KinesisImageMetadata.msg"
  DEPENDENCIES diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
