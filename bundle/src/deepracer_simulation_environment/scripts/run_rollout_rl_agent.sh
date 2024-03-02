#!/usr/bin/env bash

set -ex

#export PYTHONPATH=${COLCON_BUNDLE_PREFIX}/usr/local/lib/python3.6/dist-packages/:$PYTHONPATH
export PYTHONUNBUFFERED=1
ROLLOUT_IDX=${1:-0}
python3.11 /opt/install/start_deepracer_node_monitor.py \
    --node_monitor_file_path /opt/install/deepracer_node_monitor_list.txt 2>&1 &


if ! python3 -m markov.rollout_worker --rollout_idx $ROLLOUT_IDX; then
    exit
fi