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

cmake_minimum_required(VERSION 2.8.3)
project(kinesis_manager)
set(KINESIS_MANAGER_VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)

find_package(aws_common REQUIRED)
if(AWSSDK_FOUND)
  set(SERVICE kinesis)
  AWSSDK_DETERMINE_LIBS_TO_LINK(SERVICE AWSSDK_LIBS_TO_LINK)
  link_directories("${AWSSDK_LIB_DIR}")
endif()

find_package(CURL REQUIRED)

## Add KVS SDK libraries
include(FindKVSSDK.cmake)
add_subdirectory(kvssdk)


#############
## Library ##
#############

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED src/stream_definition_provider.cpp 
  src/kinesis_stream_manager.cpp src/default_callbacks.cpp
  src/kinesis_video_stream_interface.cpp src/kinesis_video_producer_interface.cpp)
set(${PROJECT_NAME}_IMPORTED_LIBRARIES producer)
add_dependencies(${PROJECT_NAME} KVS_SDK_IMPORT)

foreach(_imported_lib ${${PROJECT_NAME}_IMPORTED_LIBRARIES})
  if(NOT TARGET ${_imported_lib})
    add_library(${_imported_lib} UNKNOWN IMPORTED)
    set_target_properties(${_imported_lib} PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${AWSSDK_EXTERNAL_INCLUDE_DIR}")
    set_target_properties(${_imported_lib} PROPERTIES
      IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
      IMPORTED_LOCATION "${KVSSDK_EXTERNAL_LIB_DIR}/lib${_imported_lib}.so")
    set_property(TARGET producer APPEND PROPERTY
      IMPORTED_CONFIGURATIONS RELEASE)
  endif()
endforeach()

target_include_directories(${PROJECT_NAME} PRIVATE ${aws_common_INCLUDE_DIRS} ${AWSSDK_INCLUDE_DIR} ${KVSSDK_EXTERNAL_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} ${AWSSDK_LIBS_TO_LINK} ${${PROJECT_NAME}_IMPORTED_LIBRARIES} log4cplus)


#############
## Tests ##
#############

# enable_testing()
# # Build tests which perform API calls to AWS and require user configuration set up
# if(BUILD_AWS_TESTING)
#   add_definitions(-DBUILD_AWS_TESTING)
# endif()

# find_common_test_packages()

# add_common_gtest(test_kinesis_manager
#   test/kinesis_manager_test.cpp
# )
# if(TARGET test_kinesis_manager)
#   target_include_directories(test_kinesis_manager PRIVATE
#     ${aws_common_INCLUDE_DIRS}
#     ${KVSSDK_EXTERNAL_INCLUDE_DIR}
#   )
#   target_link_libraries(test_kinesis_manager
#     ${PROJECT_NAME}
#     ${aws_common_LIBRARIES}
#     ${GMOCK_LIBRARY}
#     pthread
#   )
# endif()

# add_common_gtest(test_stream_subscription_installer
#   test/stream_subscription_installer_test.cpp
# )
# if(TARGET test_stream_subscription_installer)
#   target_include_directories(test_stream_subscription_installer PRIVATE
#     ${aws_common_INCLUDE_DIRS}
#     ${KVSSDK_EXTERNAL_INCLUDE_DIR}
#   )
#   target_link_libraries(test_stream_subscription_installer
#     ${PROJECT_NAME}
#     ${GMOCK_LIBRARY}
#     pthread
#   )
# endif()

# add_common_gtest(test_default_callbacks
#   test/default_callbacks_test.cpp
# )
# if(TARGET test_default_callbacks)
#   target_include_directories(test_default_callbacks PRIVATE
#     ${aws_common_INCLUDE_DIRS}
#     ${KVSSDK_EXTERNAL_INCLUDE_DIR}
#   )
#   target_link_libraries(test_default_callbacks
#     ${PROJECT_NAME}
#     ${aws_common_LIBRARIES}
#     ${GMOCK_LIBRARY}
#     pthread
#   )
# endif()

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

## Export KVS SDK artifacts
install(
  DIRECTORY ${KVSSDK_EXTERNAL_LIB_DIR}/
  DESTINATION lib/
)

install(
  DIRECTORY ${KVSSDK_EXTERNAL_INCLUDE_DIR}/
  DESTINATION include
)

export(TARGETS ${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake")
export(PACKAGE ${PROJECT_NAME})

CONFIGURE_FILE(${PROJECT_NAME}Config.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake @ONLY)
CONFIGURE_FILE(${PROJECT_NAME}ConfigVersion.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake @ONLY)

INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(EXPORT ${PROJECT_NAME}-targets DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES FindKVSSDK.cmake DESTINATION share/${PROJECT_NAME}/cmake)
