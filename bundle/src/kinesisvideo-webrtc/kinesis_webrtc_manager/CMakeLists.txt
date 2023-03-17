cmake_minimum_required(VERSION 3.10.3)
project(kinesis_webrtc_manager)
set(KINESIS_WEBRTC_MANAGER_VERSION 0.1.0)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(aws_common REQUIRED)
find_package(aws_ros1_common REQUIRED)

set(WEBRTCSDK_VERSION 1.0.0)

set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)

#############
## Library ##
#############
file(MAKE_DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/include)
file(MAKE_DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/lib)

include(ExternalProject)

ExternalProject_Add(WEBRTC_SDK_IMPORT
  GIT_REPOSITORY "https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c.git"
  GIT_TAG "v1.5.0"
  SOURCE_DIR ${EXTERNAL_INSTALL_LOCATION}/src
  BINARY_DIR ${EXTERNAL_INSTALL_LOCATION}/build
  PATCH_COMMAND
    # Patch amazon-kinesis-video-streams-webrtc-sdk-c
       cp ${CMAKE_CURRENT_SOURCE_DIR}/webrtc-patches/webrtc-sdk-c.patch ${EXTERNAL_INSTALL_LOCATION}/src
    && cd ${EXTERNAL_INSTALL_LOCATION}/src
    && git apply --ignore-space-change --ignore-whitespace ${EXTERNAL_INSTALL_LOCATION}/src/webrtc-sdk-c.patch
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
  libkvsCommonLws.a
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

target_include_directories(${PROJECT_NAME} PRIVATE ${WEBRTCSDK_INCLUDE_DIR} ${KINESIS_WEBRTC_MANAGER_INCLUDE_DIR} )


## Export source code as SHARED
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} ${WEBRTCSDK_LIBS})

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

# Export WEBRTC SDK artifacts
install(
  DIRECTORY ${WEBRTCSDK_INCLUDE_DIR}/
  DESTINATION include
)


install(
  DIRECTORY ${WEBRTCSDK_LIB_DIR}/
  DESTINATION lib/
)

configure_file(${PROJECT_NAME}Config.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake @ONLY
)
configure_file(${PROJECT_NAME}ConfigVersion.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake @ONLY
)
export(TARGETS ${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake"
)
export(PACKAGE ${PROJECT_NAME})

install(
  FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)
install(
  FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)
install(
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION share/${PROJECT_NAME}/cmake
)
