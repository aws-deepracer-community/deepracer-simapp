cmake_minimum_required(VERSION 2.8.3)
project(aws_ros1_common)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  aws_common
  roscpp
)

if(AWSSDK_FOUND)
  set(SERVICE core)
  AWSSDK_DETERMINE_LIBS_TO_LINK(SERVICE AWSSDK_LIBS)
  link_directories("${AWSSDK_LIB_DIR}")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_library(${PROJECT_NAME} SHARED
  src/sdk_utils/ros1_node_parameter_reader.cpp
  src/sdk_utils/logging/aws_ros_logger.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${AWSSDK_LIBS} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${AWSSDK_INCLUDE_DIR})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include/
)

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)

#   add_rostest_gtest(test_client_configuration_provider
#     test/test_client_configuration_provider.test
#     test/client_configuration_provider_test.cpp
#   )
#   target_link_libraries(test_client_configuration_provider
#     ${PROJECT_NAME}
#     ${aws_common_LIBRARIES}
#     ${catkin_LIBRARIES}
#   )

#   add_rostest_gtest(test_aws_ros_logger
#     test/sdk_utils/logging/test_aws_ros_logger.test
#     test/sdk_utils/logging/aws_ros_logger_test.cpp
#   )
#   target_link_libraries(test_aws_ros_logger
#     ${PROJECT_NAME}
#     ${aws_common_LIBRARIES}
#     ${catkin_LIBRARIES}
#   )

#   add_rostest_gtest(test_parameter_reader
#     test/test_parameter_reader.test
#     test/parameter_reader_test.cpp
#   )
#   target_link_libraries(test_parameter_reader
#     ${PROJECT_NAME}
#     ${aws_common_LIBRARIES}
#     ${catkin_LIBRARIES}
#   )
# endif()
