# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.14.4)
project(kinesis_webrtc_manager)
set(KINESIS_WEBRTC_MANAGER_VERSION 0.1.0)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aws_common REQUIRED)
find_package(aws_ros2_common REQUIRED)

set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)

#############
## Library ##
#############
file(MAKE_DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/include)
file(MAKE_DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/lib)

include(ExternalProject)

ExternalProject_Add(WEBRTC_SDK_IMPORT
  GIT_REPOSITORY "https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c.git"
  GIT_TAG "v1.12.0"
  SOURCE_DIR ${EXTERNAL_INSTALL_LOCATION}/src
  BINARY_DIR ${EXTERNAL_INSTALL_LOCATION}/build
  # GCC 15 (Ubuntu 26.04) defaults to C23 and makes implicit function declarations an error.
  # The SDK and its dependencies (kvspic) are written for gnu17 and call GNU extensions such as
  # pthread_getname_np without defining _GNU_SOURCE.
  CMAKE_ARGS "-DCMAKE_C_FLAGS=-Wno-enum-int-mismatch -std=gnu17 -D_GNU_SOURCE"
  PATCH_COMMAND ${CMAKE_COMMAND} -E echo "Checking if patch needs to be applied" && 
              (git -C ${EXTERNAL_INSTALL_LOCATION}/src apply --check --ignore-space-change --ignore-whitespace ${CMAKE_CURRENT_SOURCE_DIR}/webrtc-patches/webrtc-sdk-c.patch > /dev/null 2>&1 && 
               ${CMAKE_COMMAND} -E echo "Applying patch" && 
               git -C ${EXTERNAL_INSTALL_LOCATION}/src apply --ignore-space-change --ignore-whitespace ${CMAKE_CURRENT_SOURCE_DIR}/webrtc-patches/webrtc-sdk-c.patch || 
               ${CMAKE_COMMAND} -E echo "webrtc-sdk-c.patch does not apply to this SDK version (known; see prepare-webrtc-sdk), not applied")
              # System OpenSSL instead of a source-built OpenSSL 1.1.1, and no libwebsockets
              # minimal examples (rejected by CMake 4). See the script for details.
              && ${CMAKE_CURRENT_SOURCE_DIR}/prepare-webrtc-sdk ${EXTERNAL_INSTALL_LOCATION}/src
  INSTALL_COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/install-webrtc ${EXTERNAL_INSTALL_LOCATION}
    && chmod +x ${EXTERNAL_INSTALL_LOCATION}/install-webrtc
    && ${EXTERNAL_INSTALL_LOCATION}/install-webrtc ${EXTERNAL_INSTALL_LOCATION}
)

###############
## "Install" ##
###############

set(WEBRTCSDK_INCLUDE_DIR ${CMAKE_BINARY_DIR}/external/include)
set(WEBRTCSDK_LIB_DIR ${CMAKE_BINARY_DIR}/external/lib)
set(WEBRTCSDK_LIBS
  libkvsWebrtcClient.so
  libkvsWebrtcSignalingClient.so
  libkvsCommonLws.so
)

foreach(_imported_lib ${WEBRTCSDK_LIBS})
  if(NOT TARGET ${_imported_lib})
    add_library(${_imported_lib} UNKNOWN IMPORTED GLOBAL)
    set_target_properties(${_imported_lib} PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES ${WEBRTCSDK_INCLUDE_DIR}
      IMPORTED_CONFIGURATIONS RELEASE
      IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
      IMPORTED_LOCATION ${WEBRTCSDK_LIB_DIR}/${_imported_lib}
    )
  endif()
endforeach()

set(KINESIS_WEBRTC_MANAGER_INCLUDE_DIR 
  include/common.h
  include/kinesis_webrtc_client_interface.h
  include/kinesis_webrtc_manager.h
  include/kinesis_webrtc_utils.h
)

###########
## Build ##
###########

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED src/kinesis_webrtc_utils.cpp
                                   src/kinesis_webrtc_client_interface.cpp
                                   src/kinesis_webrtc_manager.cpp)

add_dependencies(${PROJECT_NAME} WEBRTC_SDK_IMPORT)

# Use target_include_directories with proper visibility
target_include_directories(${PROJECT_NAME} PRIVATE ${WEBRTCSDK_INCLUDE_DIR})

# Export source code as SHARED with proper BUILD_INTERFACE and INSTALL_INTERFACE
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${WEBRTCSDK_INCLUDE_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME} 
  ${WEBRTCSDK_LIBS} 
  ${aws_common_LIBRARIES}
)

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
  DESTINATION include/${PROJECT_NAME}
)

# Export WEBRTC SDK artifacts
install(
  DIRECTORY ${WEBRTCSDK_INCLUDE_DIR}/
  DESTINATION include/${PROJECT_NAME}
)


install(
  DIRECTORY ${WEBRTCSDK_LIB_DIR}/
  DESTINATION lib/
)

export(TARGETS ${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake"
)
export(PACKAGE ${PROJECT_NAME})

install(
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION share/${PROJECT_NAME}/cmake
)
# Export dependencies for ROS 2
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}-targets)
ament_export_dependencies(aws_common aws_ros2_common)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_kinesis_webrtc_manager
    test/kinesis_webrtc_manager_test.cpp
    TIMEOUT 10)  # Set a shorter timeout for faster test runs
  
  target_include_directories(test_kinesis_webrtc_manager PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${WEBRTCSDK_INCLUDE_DIR}>")
  target_link_libraries(test_kinesis_webrtc_manager
    ${PROJECT_NAME}
    ${WEBRTCSDK_LIBS}
    ${aws_common_LIBRARIES}
  )

  ament_add_gmock(test_kinesis_webrtc_utils
    test/kinesis_webrtc_utils_test.cpp)

  target_include_directories(test_kinesis_webrtc_utils PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${WEBRTCSDK_INCLUDE_DIR}>")
  target_link_libraries(test_kinesis_webrtc_utils
    ${PROJECT_NAME}
    ${WEBRTCSDK_LIBS}
    ${aws_common_LIBRARIES}
  )


endif()

# ament_package() generates share/kinesis_webrtc_manager/cmake/kinesis_webrtc_managerConfig.cmake.
# The package used to install a second file under that name, so which one was installed depended
# on file timestamps; its content (the WebRTC SDK libraries) is now an extras file.
ament_package(CONFIG_EXTRAS kinesis_webrtc_manager-extras.cmake.in)