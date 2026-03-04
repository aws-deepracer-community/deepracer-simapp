#!/usr/bin/env bash

# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

set -e

export PYTHONUNBUFFERED=1
ROLLOUT_IDX=${1:-0}
python3 /opt/amazon/script/start_deepracer_node_monitor.py \
    --node_monitor_file_path /opt/amazon/script/config/deepracer_node_monitor_list.txt 2>&1 &

if ! python3 -m markov.rollout_worker --rollout_idx $ROLLOUT_IDX; then
    exit
fi