#!/usr/bin/env bash

set -ex

#export PYTHONPATH=${COLCON_BUNDLE_PREFIX}/usr/local/lib/python3.6/dist-packages/:$PYTHONPATH
export PYTHONUNBUFFERED=1

python3 /opt/amazon/script/start_deepracer_node_monitor.py \
    --node_monitor_file_path /opt/amazon/src/deepracer_node_monitor/config/deepracer_node_monitor_list.txt 2>&1 &

python3 -m markov.evaluation_worker