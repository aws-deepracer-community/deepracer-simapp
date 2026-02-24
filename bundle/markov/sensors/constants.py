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

'''Constants for the sensor package.

The ``Input`` enum is the single authoritative list of supported sensor modalities.
All code that needs to identify a sensor type should import from here rather than
from the (now removed) ``markov.architecture.constants`` module.
'''
from enum import Enum

# ---------------------------------------------------------------------------
# Sensor type identifiers
# ---------------------------------------------------------------------------

class Input(Enum):
    '''All sensor inputs supported by the DeepRacer Gazebo environment.

    Values are the string keys used in ROS topic names and in observation-space
    dictionaries returned by ``gymnasium.Env.reset()`` / ``step()``.
    '''
    OBSERVATION             = 'observation'
    LIDAR                   = 'LIDAR'
    # BINARY_SECTOR_LIDAR — coarse binary sector representation
    SECTOR_LIDAR            = 'SECTOR_LIDAR'
    # DISCRETIZED_SECTOR_LIDAR — configurable per-sector discretisation
    DISCRETIZED_SECTOR_LIDAR = 'DISCRETIZED_SECTOR_LIDAR'
    CAMERA                  = 'FRONT_FACING_CAMERA'
    LEFT_CAMERA             = 'LEFT_CAMERA'
    STEREO                  = 'STEREO_CAMERAS'

    @classmethod
    def validate_inputs(cls, input_list):
        '''Return True if every item in *input_list* is a recognised sensor type.'''
        return all(_input in cls._value2member_map_ for _input in input_list)


# ---------------------------------------------------------------------------
# LIDAR hardware constants (must match values in racecar.launch)
# ---------------------------------------------------------------------------

LIDAR_360_DEGREE_SAMPLE              = 64
LIDAR_360_DEGREE_HORIZONTAL_RESOLUTION = 1
LIDAR_360_DEGREE_MIN_ANGLE           = -2.61799
LIDAR_360_DEGREE_MAX_ANGLE           =  2.61799
LIDAR_360_DEGREE_MIN_RANGE           =  0.15
LIDAR_360_DEGREE_MAX_RANGE           = 12.0
LIDAR_360_DEGREE_RANGE_RESOLUTION    =  0.01
LIDAR_360_DEGREE_NOISE_MEAN          =  0.0
LIDAR_360_DEGREE_NOISE_STDDEV        =  0.01
