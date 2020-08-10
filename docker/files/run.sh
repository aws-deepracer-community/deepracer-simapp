#!/usr/bin/env bash
export XAUTHORITY=/root/.Xauthority

# Ensure we have a roll-out index; also when we have only one worker
if [ -z "$ROLLOUT_IDX" ]; then
	export ROLLOUT_IDX=0
fi

# If we have multiple workers we need to do assignment of worker number
# Additionally if multi-config is enabled then the specifc WORLD and 
# YAML files must be assigned.
if [ "$1" == "multi" ]; then

	# Use Docker Swarm Replica .Task.Slot
	if [ -n "$DOCKER_REPLICA_SLOT" ]; then
		WORKER_NUM=$DOCKER_REPLICA_SLOT
	# Create an 'election file' in local file system
	else
		COMMS_FILE=/mnt/comms/workers
		echo $HOSTNAME >> $COMMS_FILE
		WORKER_NUM=$(cat -n $COMMS_FILE | grep $HOSTNAME | cut -f1)
	fi

	export ROLLOUT_IDX=$(expr $WORKER_NUM - 1 )

	# Check if multi-config has been enabled, then override S3_YAML_NAME and WORLD_NAME
	if [ -n "$MULTI_CONFIG" ]; then
		echo $MULTI_CONFIG
		export S3_YAML_NAME=$(echo $MULTI_CONFIG | jq --arg worker $ROLLOUT_IDX -r '.multi_config[$worker | tonumber ].config_file')
		export WORLD_NAME=$(echo $MULTI_CONFIG | jq --arg worker $ROLLOUT_IDX -r '.multi_config[$worker | tonumber ].world_name')
	fi

	echo "Starting as worker $ROLLOUT_IDX, using world $WORLD_NAME and configuration $S3_YAML_NAME."

fi

# If no run-option given then use the distributed training
if [ -z ${2+x} ]; then
	$2 = "distributed_training.launch"
	exit

fi

# Initialize ROS & the Bundle
source /opt/ros/${ROS_DISTRO}/setup.bash
source setup.bash

# Start an X server if we do not have one
if [[ "${USE_EXTERNAL_X,,}" != "true" ]]; then
	export DISPLAY=:0 # Select screen 0 by default.
	xvfb-run -f $XAUTHORITY -l -n 0 -s ":0 -screen 0 1400x900x24" jwm &
	x11vnc -bg -forever -nopw -rfbport 5900 -display WAIT$DISPLAY &
# Ensure DISPLAY is defined
else
	if [ -z "$DISPLAY" ]; then
		export DISPLAY=:$(ls /tmp/.X11-unix/ | cut -c2 | head -1)
	fi
fi

# Start the training
roslaunch deepracer_simulation_environment $2 &

# If GUI is desired then also start RQT and RVIZ
if [[ "${ENABLE_GUI,,}" == "true" ]];
then
	rqt &
	rviz &
fi

sleep 1

echo "IP: $(hostname -I) ($(hostname))"
wait
