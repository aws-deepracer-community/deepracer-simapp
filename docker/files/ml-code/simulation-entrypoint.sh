#!/bin/bash

# Sourcing all ROS packages
source /opt/ros/noetic/setup.bash
source /opt/simapp/setup.bash
export GAZEBO_MODEL_PATH=/opt/simapp/deepracer_simulation_environment/share/deepracer_simulation_environment/
echo "Executing simulation-entrypoint.sh script"

printenv
exec "${@:1}"
