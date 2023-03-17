cmake_minimum_required(VERSION 2.8.3)
project(kinesis_video_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    message_runtime
    diagnostic_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    KinesisVideoFrame.msg
    KinesisImageMetadata.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES diagnostic_msgs
)

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)

catkin_package()
