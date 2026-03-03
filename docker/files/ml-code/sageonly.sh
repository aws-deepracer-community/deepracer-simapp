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

echo "Starting sageonly.sh"

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

export PYTHONPATH=/opt/amazon/install/sagemaker_rl_agent/lib/python3.12/site-packages/:$PYTHONPATH

echo "Logging sagemaker and robomaker in seperate cloudwatch stream"
export SIMULATION_LOG_GROUP=/aws/deepracer/${JOB_TYPE}/SimulationJobs
export TRAINING_LOG_GROUP=/aws/deepracer/${JOB_TYPE}/TrainingJobs
/usr/bin/python3.12 /opt/ml/code/scripts/cloudwatch_uploader.py --cw_log_group_name ${SIMULATION_LOG_GROUP} --cw_log_stream_name ${JOB_NAME} --log_symlink_file_path /opt/ml/simapp.log &
/usr/bin/python3.12 /opt/ml/code/scripts/cloudwatch_uploader.py --cw_log_group_name ${TRAINING_LOG_GROUP} --cw_log_stream_name ${JOB_NAME} --log_symlink_file_path /opt/ml/training.log &

# Starting sagemaker instance inside conda environment
export PATH="/root/anaconda/bin:/root/anaconda/condabin:/root/anaconda/bin:/opt/ml:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
conda init bash
source /root/anaconda/etc/profile.d/conda.sh
source /root/.bashrc
conda activate sagemaker_env
# Start the redis server and Coach training worker
redis-server /etc/redis/redis.conf &
sleep 5 
echo "Running training job inside conda on single sagemaker instance..."
echo "Input argument to training worker $@"
# redirect stderr to stdout and have error messages sent to the same file as standard output
export DR_MARKOV_WORLD_CONFIG_LOCAL_YAML_FILE=/opt/ml/code/custom_files/training_params.yaml
python3.12 /opt/amazon/install/sagemaker_rl_agent/lib/python3.12/site-packages/markov/training_worker.py $@ > /opt/ml/training.log 2>&1 &
conda deactivate

export PATH="/opt/ml/:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/amazon/install/setup.bash
export DISPLAY=:0 # Select screen 0 by default.
export GAZEBO_MODEL_PATH=./deepracer_simulation_environment/share/deepracer_simulation_environment
xvfb-run -f $XAUTHORITY -l -n 0 -s ":0 -screen 0 1400x900x24" jwm &
echo "Running simulation job on single sagemaker instance..."
echo "Check ${SIMULATION_LOG_GROUP} and ${TRAINING_LOG_GROUP} for training and simulation logs."
# redirect stderr to stdout and have error messages sent to the same file as standard output
set +e
ros2 launch deepracer_simulation_environment $SIMULATION_LAUNCH_FILE > /opt/ml/simapp.log 2>&1
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
