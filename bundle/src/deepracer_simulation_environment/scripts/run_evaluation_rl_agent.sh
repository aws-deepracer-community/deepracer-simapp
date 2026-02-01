#!/usr/bin/env bash

# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

set -e
export PYTHONUNBUFFERED=1

echo "Starting DeepRacer evaluation agent..."

# Start node monitor in background
echo "Starting DeepRacer node monitor..."
python3 /opt/amazon/script/start_deepracer_node_monitor.py \
    --node_monitor_file_path /opt/amazon/script/config/deepracer_node_monitor_list.txt 2>&1 &

echo "Waiting for system to stabilize..."
sleep 5

echo "Starting markov evaluation worker..."

# ESSENTIAL: Source ROS setup before activating conda to ensure ROS is available
source /opt/ros/jazzy/setup.bash
source /opt/amazon/install/setup.bash
export PYTHONPATH=/opt/amazon:$PYTHONPATH
source /root/anaconda/bin/activate sagemaker_env

# Run the evaluation worker
python3 -m markov.evaluation_worker

echo "Markov evaluation worker completed successfully"
echo "DeepRacer evaluation agent finished with exit code: $?"
