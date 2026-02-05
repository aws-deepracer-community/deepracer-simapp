#!/bin/bash

#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

ROS_DISTRO=jazzy
# # aws_common ROS package used in kinesis is outdated and wants aws credentials
# # in /home/robomaker//.aws/config location
mkdir -p /root/.aws/
ln -s ${AWS_CONFIG_FILE} /root/.aws/credentials

# Sourcing all ROS packages
source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/amazon/install/setup.bash
export GAZEBO_MODEL_PATH=/opt/amazon/install/deepracer_simulation_environment/share/deepracer_simulation_environment/
echo "Executing simulation-entrypoint.sh script"

printenv
exec "${@:1}"
