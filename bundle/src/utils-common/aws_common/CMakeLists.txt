cmake_minimum_required(VERSION 3.0.2)
project(aws_common)

SET(AWS_COMMON_VERSION 1.0.0)

add_compile_options(-std=c++11)
if (DEFINED ENV{ROS_DISTRO})
    add_definitions(-DCMAKE_ROS_DISTRO="$ENV{ROS_DISTRO}")
else()
    add_definitions(-DCMAKE_ROS_DISTRO="unknown")
endif()
if (DEFINED ENV{ROS_VERSION})
    add_definitions(-DCMAKE_ROS_VERSION="$ENV{ROS_VERSION}")
else()
    add_definitions(-DCMAKE_ROS_VERSION="1")
endif()

include(GNUInstallDirs)
if(NOT EXTERNAL_INSTALL_LOCATION)
  set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)
endif()

## Add AWS SDK libraries
set(SERVICE "core" "logs" "monitoring" "s3" "kinesis" "iot" "lex" "polly")
foreach(_service ${SERVICE})
  list(APPEND ${PROJECT_NAME}_IMPORTED_LIBRARIES "aws-cpp-sdk-${_service}")
endforeach()
add_subdirectory(awssdk)

include_directories(include)
## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/sdk_utils/parameter_path.cpp
  src/sdk_utils/aws_profile_provider.cpp
  src/sdk_utils/client_configuration_provider.cpp
  src/sdk_utils/logging/aws_log_system.cpp
  src/sdk_utils/auth/service_credentials_provider.cpp
  src/fs_utils/wordexp_ros.cpp)
add_dependencies(${PROJECT_NAME} AWS_SDK_IMPORT)

foreach(_imported_lib ${${PROJECT_NAME}_IMPORTED_LIBRARIES})
  if(NOT TARGET ${_imported_lib})
    add_library(${_imported_lib} UNKNOWN IMPORTED)
    set_target_properties(${_imported_lib} PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${EXTERNAL_INSTALL_LOCATION}/include")
    set_target_properties(${_imported_lib} PROPERTIES
      IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
      IMPORTED_LOCATION "${EXTERNAL_INSTALL_LOCATION}/${CMAKE_INSTALL_LIBDIR}/lib${_imported_lib}.so")
    set_property(TARGET ${_imported_lib} APPEND PROPERTY
      IMPORTED_CONFIGURATIONS RELEASE)
  endif()
endforeach()

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${${PROJECT_NAME}_IMPORTED_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${AWSSDK_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


#############
### Tests ###
#############
# enable_testing()
# include(${CMAKE_SOURCE_DIR}/cmake/DefineTestMacros.cmake)
# find_common_test_packages()
# add_common_gtest(test_aws_log_system test/sdk_utils/logging/aws_log_system_test.cpp)
# add_common_gtest(test_throttling_manager test/sdk_utils/throttling_manager_test.cpp)
# add_common_gtest(test_client_configuration_provider test/sdk_utils/client_configuration_provider_test.cpp)
# add_common_gtest(test_service_credentials_provider test/sdk_utils/auth/service_credentials_provider_test.cpp)
# add_common_gtest(test_wordexp_ros test/fs_utils/wordexp_ros_test.cpp)

# add_library(test_utils test/sdk_utils/parameter_reader_mock.cpp)
# target_include_directories(test_utils PUBLIC ${PROJECT_SOURCE_DIR}/test/include)

# set(LIBS_FOR_TESTS
#   ${PROJECT_NAME}
#   test_utils
#   ${aws_common_LIBRARIES}
#   ${GMOCK_LIBRARY}
#   pthread
# )

# set(HEADERS_FOR_TESTS
#   ${aws_common_INCLUDE_DIRS}
#   ${AWSSDK_INCLUDE_DIR}
# )

# macro(link_test_target target_name)
#   if(TARGET ${target_name})
#     target_include_directories("${target_name}" PRIVATE ${HEADERS_FOR_TESTS})
#     target_link_libraries("${target_name}" ${LIBS_FOR_TESTS})
#   endif()
# endmacro()

# link_test_target(test_aws_log_system)
# link_test_target(test_throttling_manager)
# link_test_target(test_client_configuration_provider)
# link_test_target(test_service_credentials_provider)
# link_test_target(test_wordexp_ros)

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
  DESTINATION include/
)

export(TARGETS ${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake")
export(PACKAGE ${PROJECT_NAME})

CONFIGURE_FILE(${PROJECT_NAME}Config.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake @ONLY)
CONFIGURE_FILE(${PROJECT_NAME}ConfigVersion.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake @ONLY)
INSTALL(DIRECTORY ${PROJECT_BINARY_DIR}/external/${CMAKE_INSTALL_LIBDIR}/cmake/AWSSDK/ DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES ${CMAKE_SOURCE_DIR}/cmake/DefineTestMacros.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(EXPORT ${PROJECT_NAME}-targets DESTINATION share/${PROJECT_NAME}/cmake)
