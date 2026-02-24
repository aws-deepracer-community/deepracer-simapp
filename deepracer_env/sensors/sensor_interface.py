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

'''Interfaces for sensors used in the DeepRacer Gazebo environment.

Sensor objects bridge ROS topic subscriptions to gymnasium-compatible
observation spaces.  Concrete implementations live in sensors_rollout.py.
'''
import abc
from abc import ABCMeta


class SensorInterface(object, metaclass=abc.ABCMeta):
    '''Abstract base for all DeepRacer sensors.

    Each concrete sensor must:
    - expose a ``gymnasium.spaces.Space`` via :meth:`get_observation_space`
    - return the latest reading as a plain ``dict`` via :meth:`get_state`
    - support resetting its internal buffer via :meth:`reset`
    '''

    @abc.abstractmethod
    def get_observation_space(self):
        '''Return a ``gymnasium.spaces.Space`` describing this sensor\'s output.'''
        raise NotImplementedError('Sensor must implement get_observation_space')

    @abc.abstractmethod
    def get_state(self, block=True):
        '''Return the latest sensor reading as a ``dict``.

        Args:
            block (bool): If True, block until a new sample is available.

        Returns:
            dict: mapping from sensor key (``Input.value``) to ``np.ndarray``.
        '''
        raise NotImplementedError('Sensor must implement get_state')

    @abc.abstractmethod
    def reset(self):
        '''Flush the internal double-buffer so the next :meth:`get_state`
        call will return fresh data from Gazebo.
        '''
        raise NotImplementedError('Sensor must implement reset')


class LidarInterface(SensorInterface, metaclass=ABCMeta):
    '''Marker interface for LIDAR-based sensors.

    The composite sensor uses this to apply special non-blocking read
    semantics for LIDAR (its lower 10 Hz update rate would stall the
    control loop if always read with ``block=True``).
    '''
    pass
