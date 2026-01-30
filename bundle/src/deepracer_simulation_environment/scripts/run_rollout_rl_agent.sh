#!/usr/bin/env bash

# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

set -e

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source /opt/amazon/install/setup.bash

# Activate conda environment
source /root/anaconda/etc/profile.d/conda.sh
conda activate sagemaker_env

export PYTHONUNBUFFERED=1
ROLLOUT_IDX=${1:-0}
python3 /opt/simapp/start_deepracer_node_monitor.py \
    --node_monitor_file_path /opt/simapp/deepracer_node_monitor_list.txt 2>&1 &


if ! python3 -m markov.rollout_worker --rollout_idx $ROLLOUT_IDX; then
    exit
fi