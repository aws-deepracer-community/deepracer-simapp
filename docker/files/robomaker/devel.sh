#!/usr/bin/env bash

# Create directory
mkdir -p $XDG_RUNTIME_DIR ~/.gazebo
touch ~/.gazebo/gui.ini

# Initialize ROS & the Bundle
export IGN_IP=127.0.0.1
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/simapp/setup.bash
export GAZEBO_RESOURCE_PATH=/opt/simapp/deepracer_simulation_environment/share/deepracer_simulation_environment
export GAZEBO_MODEL_PATH=/opt/simapp/deepracer_simulation_environment/share/deepracer_simulation_environment

echo "IP: $(hostname -I) ($(hostname))"

source /usr/share/gazebo/setup.sh
gazebo /opt/simapp/deepracer_simulation_environment/share/deepracer_simulation_environment/worlds/${DR_WORLD_NAME}.world --verbose

wait
