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
project(h264_encoder_core)

set(CMAKE_CXX_STANDARD 17)

find_package(aws_common REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED libavcodec libavformat libavutil libswscale)

#############
## Compile ##
#############

# add the publisher example
add_library(${PROJECT_NAME} SHARED
  src/h264_encoder.cpp
  src/h264_encoder_node_config.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE include
  PRIVATE ${aws_common_INCLUDE_DIRS}
  PRIVATE ${FFMPEG_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  avcodec
  avutil
  swscale
  dl
  ${FFMPEG_LIBRARIES}
)


#############
## Tests ##
#############

enable_testing()
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
include(${aws_common_DIR}/DefineTestMacros.cmake)
add_common_gtest(test_h264_encoder_core
  test/h264_encoder_test.cpp
)
if(TARGET test_h264_encoder_core)
  target_include_directories(test_h264_encoder_core PRIVATE
    include
    ${aws_common_INCLUDE_DIRS}
  )
  target_link_libraries(test_h264_encoder_core
    ${PROJECT_NAME}
    ${aws_common_LIBRARIES}
    ${GMOCK_LIBRARY}
    pthread
  )
endif()


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT "${PROJECT_NAME}-targets"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include
)

export(TARGETS ${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake")
export(PACKAGE ${PROJECT_NAME})

CONFIGURE_FILE(commonConfig.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake @ONLY)
CONFIGURE_FILE(commonConfigVersion.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake @ONLY)

INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(EXPORT ${PROJECT_NAME}-targets DESTINATION share/${PROJECT_NAME}/cmake)