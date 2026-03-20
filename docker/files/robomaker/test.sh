#!/usr/bin/env bash
export DISPLAY=:1
export XAUTHORITY=~/.Xauthority
export DEEPRACER_JOB_TYPE_ENV="LOCAL"
export WORLD_NAME="reinvent_base"
export MODEL_S3_PREFIX="LLM-Test-Model-1"
export MODEL_S3_BUCKET="X"
export APP_REGION="eu-central-1"
export AWS_REGION="eu-central-1"
export AWS_ACCESS_KEY_ID="$(aws configure get aws_access_key_id --profile default)"
export AWS_SECRET_ACCESS_KEY="$(aws configure get aws_secret_access_key --profile default)"
export ENABLE_KINESIS=True
export KINESIS_VIDEO_STREAM_NAME=
export ENABLE_GUI=False
export S3_YAML_NAME=evaluation_params.yaml
# export S3_ENDPOINT_URL=http://lserver:9000
export ROS_MASTER_URI=http://127.0.0.1:11311/
export ROS_IP=127.0.0.1
export ROS_DISTRO=noetic
# If no run-option given then use the distributed training
if [ -z ${2+x} ]; then
	$2 = "distributed_training.launch"
	exit

fi

# Initialize ROS & the Bundle
export IGN_IP=127.0.0.1
source /opt/ros/${ROS_DISTRO}/setup.bash
source setup.bash

# Start an X server if we do not have one
#export DISPLAY=:99 # Select screen 0 by default.
#xvfb-run -f $XAUTHORITY -l -s ":99 -screen 0 1400x900x24" jwm &
#x11vnc -bg -forever -nopw -rfbport 5900 -display WAIT$DISPLAY &

# Start the training
roslaunch deepracer_simulation_environment $2 &

sleep 1

echo "IP: $(hostname -I) ($(hostname))"

wait
