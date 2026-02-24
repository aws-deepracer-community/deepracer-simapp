#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

'''Utility helpers for constructing per-sensor ``gymnasium.spaces`` objects.

All rl_coach ``StateSpace``/``ImageObservationSpace``/``VectorObservationSpace``
calls have been replaced with their standard ``gymnasium.spaces`` equivalents.
'''
import numpy as np
import gymnasium
from gymnasium import spaces

from markov.environments.constants import (
    TRAINING_IMAGE_SIZE, TRAINING_LIDAR_SIZE,
    NUMBER_OF_LIDAR_SECTORS
)
from markov.sensors.constants import Input
from markov.log_handler.deepracer_exceptions import GenericError


def get_observation_space(sensor, model_metadata=None):
    '''Return a ``gymnasium.spaces.Space`` for the given sensor type.

    Args:
        sensor (str): One of the ``Input`` enum *values* (e.g. ``Input.CAMERA.value``).
        model_metadata (dict | None): Required only for
            ``DISCRETIZED_SECTOR_LIDAR``; must contain
            ``{"lidar_config": {"num_sectors": N, "num_values_per_sector": M}}``.

    Returns:
        gymnasium.spaces.Space
    '''
    if not isinstance(sensor, str):
        raise GenericError('Sensor type must be a str, got: {}'.format(type(sensor)))

    H, W = TRAINING_IMAGE_SIZE[1], TRAINING_IMAGE_SIZE[0]

    if sensor in (Input.CAMERA.value, Input.OBSERVATION.value, Input.LEFT_CAMERA.value):
        # RGB image: (H, W, 3)  uint8  [0, 255]
        box = spaces.Box(low=0, high=255, shape=(H, W, 3), dtype=np.uint8)

    elif sensor == Input.STEREO.value:
        # Stereo greyscale pair: (H, W, 2)  uint8  [0, 255]
        box = spaces.Box(low=0, high=255, shape=(H, W, 2), dtype=np.uint8)

    elif sensor == Input.LIDAR.value:
        # 64-ray distance scan  float32  [0.15 m, 1.0 m]
        box = spaces.Box(low=0.15, high=1.0,
                         shape=(TRAINING_LIDAR_SIZE,), dtype=np.float32)

    elif sensor == Input.SECTOR_LIDAR.value:
        # Binary per-sector obstacle flags  float32  [0, 1]
        box = spaces.Box(low=0.0, high=1.0,
                         shape=(NUMBER_OF_LIDAR_SECTORS,), dtype=np.float32)

    elif sensor == Input.DISCRETIZED_SECTOR_LIDAR.value:
        if model_metadata is None:
            raise GenericError(
                'model_metadata required to build DISCRETIZED_SECTOR_LIDAR space')
        lidar_config = model_metadata['lidar_config']
        n_sectors    = lidar_config['num_sectors']
        vals_per_sec = lidar_config['num_values_per_sector']
        box = spaces.Box(low=0.0, high=1.0,
                         shape=(n_sectors * vals_per_sec,), dtype=np.float32)

    else:
        raise Exception('No observation space defined for sensor: {}'.format(sensor))

    # Wrap in a Dict so the CompositeSensor can merge sub-spaces by key
    return spaces.Dict({sensor: box})

