#!/usr/bin/env bash

# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

set -e

#export PYTHONPATH=${COLCON_BUNDLE_PREFIX}/usr/local/lib/python3.6/dist-packages/:$PYTHONPATH
export PYTHONUNBUFFERED=1

python3 /opt/amazon/script/start_deepracer_node_monitor.py \
    --node_monitor_file_path /opt/amazon/src/deepracer_node_monitor/config/deepracer_virtual_event_node_monitor_list.txt 2>&1 &

python3 -m markov.virtual_event_worker