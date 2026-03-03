#!/bin/bash
# Source ROS base and the colcon simapp overlay so every child process
# (roslaunch, python3, etc.) inherits the correct PYTHONPATH and ROS_PACKAGE_PATH.
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/simapp/setup.bash

exec "$@"
