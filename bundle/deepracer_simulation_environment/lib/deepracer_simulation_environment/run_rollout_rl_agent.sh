#!/usr/bin/env bash

set -ex

#export PYTHONPATH=${COLCON_BUNDLE_PREFIX}/usr/local/lib/python3.5/dist-packages/:$PYTHONPATH
export PYTHONUNBUFFERED=1
ROLLOUT_IDX=${1:-0}

if ! python3 -m markov.rollout_worker --rollout_idx $ROLLOUT_IDX; then
    exit
fi