#!/bin/bash
set -e
set -x

echo "Building DeepRacer ROS packages"

ROS_DISTRO=noetic
ROOT_DIR=/opt/amazon
export ROS_VERSION=1

cd $ROOT_DIR
source /opt/ros/$ROS_DISTRO/setup.bash

echo "Installing ros dependencies"
rosdep install --from-paths src --ignore-src -y
rm -rf build/ install/
echo "Doing colcon build"
colcon build --cmake-args "-DSETUPTOOLS_DEB_LAYOUT=OFF" "-DPYTHON_EXECUTABLE=/usr/bin/python3" "-DPYTHON_INCLUDE_DIR=/usr/include/python3.8m"
