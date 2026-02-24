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

'''This module contains model metadata key constants'''

from enum import Enum


class ActionSpaceTypes(Enum):
    '''Enum containing the action space type keys in model metadata.'''
    DISCRETE = 'discrete'
    CONTINUOUS = 'continuous'

    @classmethod
    def has_action_space(cls, action_space):
        '''Returns True if the action space type is supported, False otherwise
            action_space - String containing action space type to check
        '''
        return action_space in cls._value2member_map_


class TrainingAlgorithm(Enum):
    '''Enum containing the training algorithm values passed as a parameter in model_metadata.'''
    CLIPPED_PPO = 'clipped_ppo'
    SAC = 'sac'

    @classmethod
    def has_training_algorithm(cls, training_algorithm):
        '''Returns True if the training algorithm is supported, False otherwise
            training_algorithm - String containing training algorithm to check
        '''
        return training_algorithm in cls._value2member_map_


class ModelMetadataKeys(Enum):
    '''This enum defines the keys used in the model metadata json
    '''
    SENSOR = 'sensor'
    NEURAL_NETWORK = 'neural_network'
    VERSION = 'version'
    ACTION_SPACE = 'action_space'
    ACTION_SPACE_TYPE = 'action_space_type'
    TRAINING_ALGORITHM = 'training_algorithm'
    STEERING_ANGLE = 'steering_angle'
    SPEED = 'speed'
    LOW = 'low'
    HIGH = 'high'
    LIDAR_CONFIG = 'lidar_config'
    NUM_SECTORS = 'num_sectors'
    NUM_VALUES_PER_SECTOR = 'num_values_per_sector'
    CLIPPING_DISTANCE = 'clipping_dist'


########################################
# Discretized Sector Lidar Default config #
########################################
class DiscretizedSectorLidarDefaults(object):
    # Number of lidar sectors
    # Allowed values: 1, 2, 4, 8, 16, 32, 64
    NUMBER_OF_SECTORS = 8
    # Max clipping distance for discretized sector lidar
    # Allowed values: 0.15 ~ 12.0
    CLIPPING_DIST = 2.0
    # Number of values in each sector for discretized sector lidar
    # Allowed values: >= 1
    NUM_VALUES_PER_SECTOR = 8
