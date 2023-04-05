#!/usr/bin/env bash

set -ex

#export PYTHONPATH=${COLCON_BUNDLE_PREFIX}/usr/local/lib/python3.6/dist-packages/:$PYTHONPATH
export PYTHONUNBUFFERED=1

if [[ -n "${HEARTBEAT_S3_LOCATION}" || "${HEARTBEAT_CLOUDWATCH_METRICS}" == "1" ]]; then
    python3 /opt/amazon/script/start_deepracer_node_monitor.py \
        --node_monitor_file_path /opt/amazon/src/deepracer_node_monitor/config/deepracer_virtual_event_node_monitor_list.txt 2>&1 &
fi

python3 -m markov.virtual_event_worker