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

'''This module contains a composite sensor class for supporting agents with multiple sensors'''
from rl_coach.spaces import StateSpace

from markov.sensors.sensor_interface import SensorInterface, LidarInterface


class CompositeSensor(SensorInterface):
    '''This class represents a composite sensor so that from the point of view of each agent there
       is only one sensor interface
    '''
    def __init__(self):
        self.sensors = list()

    def add_sensor(self, sensor):
        '''Adds a sensor to the sensor list
           sensor - Sensor object to add to the sensor list
        '''
        self.sensors.append(sensor)

    def get_observation_space(self):
        observation_space = StateSpace({})
        for sensor in self.sensors:
            observation_space.sub_spaces.update(sensor.get_observation_space().sub_spaces)
        return observation_space

    def get_state(self, block=True):
        state = dict()

        # For blocking requests, run a blocking call on each sensor
        if block:
            for sensor in self.sensors:
                # Lidar sensor update rate is 10 hz while camera sensor update rate is 15 hz.
                # Due to slower Lidar sensor update rate, if the Lidar sensor is used,
                # Lidar sensor data retrieval becomes bottleneck and makes the inference period to 10 hz.
                # The latest Lidar sensor type is sector-lidar, due to limited number of sectors and binary type
                # for sector-lidar state data, it is unlikely, that sector Lidar data change every steps.
                # Thus, it is bit unnecessary to wait for Lidar data and slow everything down.
                # We ignore blocking request to Lidar sensor update and follow-up non-blocking call below
                # will use the latest Lidar data whether it was used previously or not.
                if isinstance(sensor, LidarInterface):
                    continue
                state.update(sensor.get_state(block=block))

        # For all requests, follow-up with a non-blocking call
        # This will ensure we have the latest for all sensors in the event that one of the
        # earlier sensors in the list published new data while waiting on a blocking call above
        for sensor in self.sensors:
            state.update(sensor.get_state(block=False))

        return state

    def get_raw_state(self):
        raw_data = dict()
        for sensor in self.sensors:
            if hasattr(sensor, 'raw_data'):
                raw_data[sensor.sensor_type] = sensor.raw_data
        return raw_data

    def reset(self):
        [sensor.reset() for sensor in self.sensors]

    def get_input_embedders(self, network_type):
        input_embedders = dict()
        for sensor in self.sensors:
            input_embedders = dict(input_embedders, **sensor.get_input_embedders(network_type))
        return input_embedders
