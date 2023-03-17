cmake_minimum_required(VERSION 3.10.3)
project(kinesis_webrtc_streamer)
# Default to C++14
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(kinesis_webrtc_manager REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
             aws_common 
             aws_ros1_common 
             geometry_msgs 
             kinesis_video_msgs
             roscpp 
             std_msgs)

catkin_package()

###########
## Build ##
###########

## Specify libraries to link a library or executable target against





## Declare a C++ executable
set(KINESIS_WEBRTC_STREAMER_SRC 
    src/streamer_params.cpp
    src/subscriber_callbacks.cpp
    src/webrtc_node.cpp
)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/main.cpp ${KINESIS_WEBRTC_STREAMER_SRC})
add_library(${PROJECT_NAME}_lib ${KINESIS_WEBRTC_STREAMER_SRC})

## Specify include directories
target_include_directories(${PROJECT_NAME} PRIVATE include ${catkin_INCLUDE_DIRS} ${kinesis_webrtc_manager_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}_lib PUBLIC include ${catkin_INCLUDE_DIRS} ${kinesis_webrtc_manager_INCLUDE_DIRS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${kinesis_webrtc_manager_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${kinesis_webrtc_manager_LIBRARIES})

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
