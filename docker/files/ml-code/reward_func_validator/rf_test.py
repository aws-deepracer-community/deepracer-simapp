# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

# type: ignore
"""
How to use:
1. Docker run the image: docker run -it <image_id> bash
2. Run the flask app - validator.py in the directory /opt/amazon
    1. export FLASK_APP=validator.py
    2. python -m flask run


3. Check for running container in another CDD terminal: docker container ls
4. Open another interactive shell in exec mode for the running container: docker exec -it <container_id> bash
5. Run $pip install requests
6. Run $python3 rf_test.py
"""

from os import walk

import requests

VALIDATION_ENDPOINT = "http://127.0.0.1:5000/validate"

tracks = next(walk("/opt/amazon/routes"), (None, None, []))[2]

for t in tracks:
    track = t[:-4]
    print(track)

    # Success RFs

    # Time trial - follow the center line (Default): Should succeed
    response = requests.post(
        VALIDATION_ENDPOINT,
        json={
            "reward_function": "import random\nimport math\nimport numpy as np\nimport shapely\ndef reward_function(params):\n    '''\n    Example of rewarding the agent to follow center line\n    '''\n    \n    # Read input parameters\n    track_width = params['track_width']\n    distance_from_center = params['distance_from_center']\n    \n    # Calculate 3 markers that are at varying distances away from the center line\n    marker_1 = 0.1 * track_width\n    marker_2 = 0.25 * track_width\n    marker_3 = 0.5 * track_width\n    \n    # Give higher reward if the car is closer to center line and vice versa\n    if distance_from_center <= marker_1:\n        reward = 1.0\n    elif distance_from_center <= marker_2:\n        reward = 0.5\n    elif distance_from_center <= marker_3:\n        reward = 0.1\n    else:\n        reward = 1e-3  # likely crashed/ close to off track\n    \n    return float(reward)",
            "track_name": f"{track}",
        },
    )
    # Time trial - stay inside the two borders: Should succeed
    response = requests.post(
        VALIDATION_ENDPOINT,
        json={
            "reward_function": "def reward_function(params):\n    '''\n    Example of rewarding the agent to stay inside the two borders of the track\n    '''\n    \n    # Read input parameters\n    all_wheels_on_track = params['all_wheels_on_track']\n    distance_from_center = params['distance_from_center']\n    track_width = params['track_width']\n    \n    # Give a very low reward by default\n    reward = 1e-3\n\n    # Give a high reward if no wheels go off the track and\n    # the agent is somewhere in between the track borders\n    if all_wheels_on_track and (0.5*track_width - distance_from_center) >= 0.05:\n        reward = 1.0\n\n    # Always return a float value\n    return float(reward)",
            "track_name": f"{track}",
        },
    )
    # Time trial - prevent zig-zag: Should succeed
    response = requests.post(
        VALIDATION_ENDPOINT,
        json={
            "reward_function": "def reward_function(params):\n    '''\n    Example of penalize steering, which helps mitigate zig-zag behaviors\n    '''\n\n    # Read input parameters\n    distance_from_center = params['distance_from_center']\n    track_width = params['track_width']\n    abs_steering = abs(params['steering_angle']) # Only need the absolute steering angle\n\n    # Calculate 3 marks that are farther and father away from the center line\n    marker_1 = 0.1 * track_width\n    marker_2 = 0.25 * track_width\n    marker_3 = 0.5 * track_width\n\n    # Give higher reward if the car is closer to center line and vice versa\n    if distance_from_center <= marker_1:\n        reward = 1.0\n    elif distance_from_center <= marker_2:\n        reward = 0.5\n    elif distance_from_center <= marker_3:\n        reward = 0.1\n    else:\n        reward = 1e-3  # likely crashed/ close to off track\n\n    # Steering penality threshold, change the number based on your action space setting\n    ABS_STEERING_THRESHOLD = 15 \n    \n    # Penalize reward if the car is steering too much\n    if abs_steering > ABS_STEERING_THRESHOLD:\n        reward *= 0.8\n    return float(reward)",
            "track_name": f"{track}",
        },
    )
