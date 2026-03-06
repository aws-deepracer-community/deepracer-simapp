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

echo "Starting sageonly_evals.sh"

set -e

COACH_EXP_NAME=sagemaker_rl

# ros jazzy
export ROS_DISTRO=jazzy
export PYTHONUNBUFFERED=1
export XAUTHORITY=/root/.Xauthority
export TF_CPP_MIN_LOG_LEVEL=3

IP_ADDRESSES=$(hostname -I)
echo "HOSTNAME -I ${IP_ADDRESSES}"

# Set space as the delimiter
IFS=' '
# Read the split words into an array based on space delimiter
read -a IPS_ADDRESS_LIST <<< "$IP_ADDRESSES"
unset IFS
export ROS_IP=127.0.0.1

export DEEPRACER_JOB_TYPE_ENV="SAGEONLY"

export PATH="/opt/ml/:$PATH"
# Unset the PYTHONPATH set by the base image to avoid conflicts with sagemaker_env
export PYTHONPATH="/opt/ml/code"

# Source the libraries
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/amazon/install/setup.bash
source /root/anaconda/bin/activate sagemaker_env

export DISPLAY=:0 # Select screen 0 by default.
export GAZEBO_MODEL_PATH=./deepracer_simulation_environment/share/deepracer_simulation_environment
xvfb-run -f $XAUTHORITY -l -n 0 -s ":0 -screen 0 1400x900x24" jwm &
echo "Running simulation job on single sagemaker instance..."
# redirect stderr to stdout and have error messages sent to the same file as standard output
set +e
ros2 launch deepracer_simulation_environment ${SIMULATION_LAUNCH_FILE}
SIM_EXIT_CODE=$?
set -e

echo "Uploading ROS Gazebo Logs"
# || true allows the upload to continue even if some of source directory doesn't exist or upload failed
aws s3 sync /root/.ros/log/ s3://${S3_ROS_LOG_BUCKET}/${JOB_NAME}/ros || true
aws s3 sync /root/.gazebo/ s3://${S3_ROS_LOG_BUCKET}/${JOB_NAME}/gazebo || true

if [ ${SIM_EXIT_CODE} -ne 0 ]; then
	echo "Terminating with error. Simulation exit code: ${SIM_EXIT_CODE}"
else
	echo "Terminating successfully. Simulation exit code: ${SIM_EXIT_CODE}"
fi
exit ${SIM_EXIT_CODE}
