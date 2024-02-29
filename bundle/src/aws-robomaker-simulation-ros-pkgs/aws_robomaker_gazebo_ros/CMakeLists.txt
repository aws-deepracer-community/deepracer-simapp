cmake_minimum_required(VERSION 3.7.2)
project(aws_robomaker_gazebo_ros)

find_package(catkin REQUIRED)
catkin_package()

catkin_add_env_hooks(worldforge.overlay SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

# Install Gazebo Scripts
install(
    PROGRAMS scripts/robomaker_gazebo_wrapper.bash
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Gazebo launch files
install(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)