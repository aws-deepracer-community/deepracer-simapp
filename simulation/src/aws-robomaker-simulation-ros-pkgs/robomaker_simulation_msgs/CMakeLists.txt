cmake_minimum_required(VERSION 2.8.3)
project(robomaker_simulation_msgs)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
)

add_message_files(
	FILES
	Tag.msg
)

add_service_files(
	FILES
	Cancel.srv
	ListTags.srv
	AddTags.srv
	RemoveTags.srv
)

generate_messages(
    DEPENDENCIES std_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs
)
