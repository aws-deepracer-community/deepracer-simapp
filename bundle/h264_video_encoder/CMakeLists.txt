cmake_minimum_required(VERSION 2.8)
project(h264_video_encoder)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  h264_encoder_core
  aws_common
  aws_ros1_common
  image_transport
  message_generation
  sensor_msgs
  kinesis_video_msgs
)

catkin_package(CATKIN_DEPENDS
  aws_ros1_common
  image_transport
  message_runtime
)


#############
## Compile ##
#############

set(h264_video_encoder_SRCS
  src/h264_video_encoder.cpp
)
set(h264_video_encoder_INCS
  include
  ${aws_common_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
set(h264_video_encoder_LIBS
  ${catkin_LIBRARIES}
  aws_common
)

# add the publisher example
add_executable(${PROJECT_NAME} src/main.cpp ${h264_video_encoder_SRCS})
add_library(${PROJECT_NAME}_lib STATIC ${h264_video_encoder_SRCS})

target_include_directories(${PROJECT_NAME} PRIVATE ${h264_video_encoder_INCS})
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${h264_video_encoder_INCS})

target_link_libraries(${PROJECT_NAME} ${h264_video_encoder_LIBS})
target_link_libraries(${PROJECT_NAME}_lib ${h264_video_encoder_LIBS})

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)


#############
## Tests ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GMock QUIET)
  if(GMOCK_FOUND)
    add_rostest_gmock(test_h264_video_encoder
      test/test_h264_video_encoder.test
      test/h264_video_encoder_test.cpp
    )
    target_link_libraries(test_h264_video_encoder
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
      ${GMOCK_BOTH_LIBRARIES}
    )
  else()
    include_directories(/usr/include/gmock /usr/src/gmock)
    add_library(${PROJECT_NAME}_libgmock SHARED /usr/src/gmock/src/gmock-all.cc)

    add_rostest_gtest(test_h264_video_encoder
      test/test_h264_video_encoder.test
      test/h264_video_encoder_test.cpp
    )
    target_link_libraries(test_h264_video_encoder
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
      ${PROJECT_NAME}_libgmock
    )
  endif()
endif()
