#!/usr/bin/env bash

export PYTHONPATH=/opt/simapp/sagemaker_rl_agent/lib/python3.8/site-packages/:$PYTHONPATH
echo "Passed Environment Variables:"
env
echo ""

# Start the redis server and Coach training worker
redis-server /opt/ml/code/redis.conf & (sleep 5 && \
python3.8 /opt/simapp/sagemaker_rl_agent/lib/python3.8/site-packages/markov/training_worker.py $@ 2>&1)