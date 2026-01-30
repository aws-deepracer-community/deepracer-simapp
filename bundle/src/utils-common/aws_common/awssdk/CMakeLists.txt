cmake_minimum_required(VERSION 3.13)
project(AWSSDK)

message(STATUS "Using system-installed AWS SDK instead of building from source")

# Find the system-installed AWS SDK
find_package(AWSSDK REQUIRED COMPONENTS ${SERVICE})
add_custom_target(AWS_SDK_IMPORT)

# Set the include directory for other targets to use
set(AWSSDK_INCLUDE_DIR ${AWSSDK_INCLUDE_DIRS} PARENT_SCOPE)

# Create the external install location directory for compatibility
if(NOT EXTERNAL_INSTALL_LOCATION)
  set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)
endif()
file(MAKE_DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/include)
file(MAKE_DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/lib)

export(PACKAGE ${PROJECT_NAME})
