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

'''Utility methods for the agent’s module'''
from typing import List, Any
from markov.architecture.constants import EmbedderType, ActivationFunctions, NeuralNetwork, Input
from markov.common import ObserverInterface
from markov.architecture.custom_architectures import DEFAULT_MIDDLEWARE, VGG_MIDDLEWARE
from markov.sensors.composite_sensor import CompositeSensor


def construct_sensor(racecar_name, observation_list, factory, model_metadata_info=None):
    '''Adds the sensors to the composite sensor based on the given observation list
       sensor - Composite sensor
       racecar_name - Name of the racecar model to scope the sensor topics
       observation_list - Observation list containg the sensor information base on architecture
       factory - Object containing the sensor factory method to use
       model_metadata_info - model metadata information
    '''
    sensor = CompositeSensor()
    if not Input.validate_inputs(observation_list):
        raise Exception('Unsupported input sensor in the observation list')
    if Input.LEFT_CAMERA.value in observation_list:
        sensor.add_sensor(factory.create_sensor(racecar_name, Input.LEFT_CAMERA.value, {}))
    if Input.STEREO.value in observation_list:
        sensor.add_sensor(factory.create_sensor(racecar_name, Input.STEREO.value, {}))
    if Input.CAMERA.value in observation_list:
        sensor.add_sensor(factory.create_sensor(racecar_name, Input.CAMERA.value, {}))
    if Input.LIDAR.value in observation_list:
        sensor.add_sensor(factory.create_sensor(racecar_name, Input.LIDAR.value, {}))
    if Input.SECTOR_LIDAR.value in observation_list:
        sensor.add_sensor(factory.create_sensor(racecar_name, Input.SECTOR_LIDAR.value, {}))
    if Input.DISCRETIZED_SECTOR_LIDAR.value in observation_list:
        sensor.add_sensor(factory.create_sensor(racecar_name, Input.DISCRETIZED_SECTOR_LIDAR.value,
                                                {"model_metadata": model_metadata_info}))
    if Input.OBSERVATION.value in observation_list:
        sensor.add_sensor(factory.create_sensor(racecar_name, Input.OBSERVATION.value, {}))
    return sensor

def get_network_settings(sensor, network):
    '''Returns a dictionary containing the network information for the agent based on the
       sensor configuration
       network - Sting of desired network topology shallow, deep, deep-deep
    '''
    try:
        is_deep = network == NeuralNetwork.DEEP_CONVOLUTIONAL_NETWORK_DEEP.value
        return {'input_embedders': sensor.get_input_embedders(network),
                'middleware_embedders': VGG_MIDDLEWARE if is_deep else DEFAULT_MIDDLEWARE,
                'embedder_type': EmbedderType.SCHEME.value,
                'activation_function': ActivationFunctions.RELU.value}
    except Exception as ex:
        raise Exception("network: {} failed to load: {}, ".format(network, ex))

class RunPhaseSubject(object):
    '''This class is sink to notify observers that the run phase has changed'''
    def __init__(self) -> None:
        self._observer_list_ = list()

    def register(self, observer: ObserverInterface) -> None:
        self._observer_list_.append(observer)

    def unregister(self, observer: ObserverInterface) -> None:
        self._observer_list_.remove(observer)

    def notify(self, data: Any) -> None:
        [observer.update(data) for observer in self._observer_list_]
