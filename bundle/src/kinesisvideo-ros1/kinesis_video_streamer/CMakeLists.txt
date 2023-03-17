cmake_minimum_required(VERSION 3.1.2)
project(kinesis_video_streamer)
set(CMAKE_CXX_STANDARD 14)

###################################
## non-catkin packages           ##
###################################
include(FindCURL)
find_package(kinesis_manager REQUIRED)

# KPL configuration
set(PRODUCER_LIBRARY producer)
set(LOG_LIBRARY log4cplus)

###################################
## catkin specific configuration ##
###################################
find_package(catkin REQUIRED COMPONENTS
        aws_common
        aws_ros1_common
        kinesis_video_msgs
        roscpp
        image_transport
        std_msgs)
catkin_package()

###########
## Build ##
###########
## Declare a C++ executable
set(KINESIS_VIDEO_STREAMER_SRC src/ros_stream_subscription_installer.cpp src/subscriber_callbacks.cpp src/streamer.cpp)
add_executable(${PROJECT_NAME} src/main.cpp ${KINESIS_VIDEO_STREAMER_SRC})
add_library(${PROJECT_NAME}_lib ${KINESIS_VIDEO_STREAMER_SRC})
## Specify include directories
target_include_directories(${PROJECT_NAME} PRIVATE include ${catkin_INCLUDE_DIRS} ${kinesis_manager_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_lib PUBLIC include ${catkin_INCLUDE_DIRS} ${kinesis_manager_INCLUDE_DIRS})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${kinesis_manager_LIBRARIES} ${PRODUCER_LIBRARY} ${LOG_LIBRARY} ${CURL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${kinesis_manager_LIBRARIES} ${PRODUCER_LIBRARY} ${CURL_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(FILES kvs_log_configuration DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Tests ##
#############

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest_gtest(test_kinesis_video_streamer
#     test/test_kinesis_video_streamer.test
#     test/kinesis_video_streamer_test.cpp
#   )
#   target_include_directories(test_kinesis_video_streamer PRIVATE
#     include
#     ${catkin_INCLUDE_DIRS}
#     ${kinesis_manager_INCLUDE_DIRS}
#   )
#   target_link_libraries(test_kinesis_video_streamer
#     ${PROJECT_NAME}_lib
#     ${catkin_LIBRARIES}
#   )

#   find_package(GMock QUIET)
#   if(GMOCK_FOUND)
#     add_rostest_gmock(test_streamer_node
#     	test/test_streamer_node.test
#     	test/streamer_node_test.cpp
#     )
#     target_include_directories(test_streamer_node PRIVATE
#       include
#       ${catkin_INCLUDE_DIRS}
#       ${kinesis_manager_INCLUDE_DIRS}
#       ${GMOCK_INCLUDE_DIRS}
#     )
#     target_link_libraries(test_streamer_node
#       ${PROJECT_NAME}_lib
#       ${catkin_LIBRARIES}
#       ${GMOCK_BOTH_LIBRARIES}
#     )
#   else()
#     include_directories(/usr/include/gmock /usr/src/gmock)
#     add_library(${PROJECT_NAME}_libgmock SHARED /usr/src/gmock/src/gmock-all.cc)

#     add_rostest_gtest(test_streamer_node
#       test/test_streamer_node.test
#       test/streamer_node_test.cpp
#     )
#     target_include_directories(test_streamer_node PRIVATE
#       include
#       ${catkin_INCLUDE_DIRS}
#       ${kinesis_manager_INCLUDE_DIRS}
#     )
#     target_link_libraries(test_streamer_node
#       ${PROJECT_NAME}_lib
#       ${catkin_LIBRARIES}
#       ${PROJECT_NAME}_libgmock
#     )
#   endif()
# endif()
