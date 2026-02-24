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

'''Composite sensor that aggregates multiple individual sensors into one.

The resulting ``observation_space`` is a ``gymnasium.spaces.Dict`` whose
keys match the ``Input.value`` strings of each constituent sensor. This
makes the composite sensor a drop-in for the ``observation_space`` attribute
of ``DeepRacerEnv``.
'''
from gymnasium import spaces

from deepracer_env.sensors.sensor_interface import SensorInterface, LidarInterface


class CompositeSensor(SensorInterface):
    '''Aggregates several sensors into a single dictionary interface.

    The ``get_observation_space()`` return value is a
    ``gymnasium.spaces.Dict`` so that the gymnasium ``Env`` can expose a
    single composite ``observation_space`` that covers all active sensors.
    '''

    def __init__(self):
        self.sensors = []

    # ------------------------------------------------------------------
    # Sensor management
    # ------------------------------------------------------------------

    def add_sensor(self, sensor):
        '''Append *sensor* to the list of active sensors.

        Args:
            sensor (SensorInterface): concrete sensor instance.
        '''
        self.sensors.append(sensor)

    # ------------------------------------------------------------------
    # SensorInterface implementation
    # ------------------------------------------------------------------

    def get_observation_space(self):
        '''Return a ``gymnasium.spaces.Dict`` covering all sensors.

        Returns:
            gymnasium.spaces.Dict
        '''
        sub_spaces = {}
        for sensor in self.sensors:
            sub_space = sensor.get_observation_space()
            # Each individual sensor returns a ``spaces.Dict({sensor_key: Box})``
            # so we can simply merge the inner dicts here.
            sub_spaces.update(sub_space.spaces)
        return spaces.Dict(sub_spaces)

    def get_state(self, block=True):
        '''Merge state dicts from all sensors into one flat dict.

        LIDAR sensors are always read with ``block=False`` to avoid
        stalling the control loop (LIDAR publishes at 10 Hz vs 15 Hz
        for cameras).  A follow-up non-blocking read on every sensor
        then ensures the freshest available data for everything.

        Args:
            block (bool): If True, block on camera-type sensors.

        Returns:
            dict: flat ``{sensor_key: np.ndarray}`` mapping.
        '''
        state = {}

        if block:
            for sensor in self.sensors:
                if isinstance(sensor, LidarInterface):
                    # LIDAR: skip blocking read – handled below
                    continue
                state.update(sensor.get_state(block=True))

        # Non-blocking pass to pick up latest data from all sensors
        # (also handles LIDAR which was skipped above)
        for sensor in self.sensors:
            state.update(sensor.get_state(block=False))

        return state

    def get_raw_state(self):
        '''Collect raw (unscaled) sensor payloads for debugging / logging.

        Returns:
            dict: ``{sensor_type: raw_data}``
        '''
        raw_data = {}
        for sensor in self.sensors:
            if hasattr(sensor, 'raw_data'):
                raw_data[sensor.sensor_type] = sensor.raw_data
        return raw_data

    def reset(self):
        '''Reset all sensors\' internal buffers.'''
        for sensor in self.sensors:
            sensor.reset()
