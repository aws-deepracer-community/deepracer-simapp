'''Utility helpers for the ``markov.agents`` package.

The RL Coach-specific helpers (get_network_settings, embedder
configuration) have been removed.  What remains is:

* construct_sensor -- build a CompositeSensor from a list of sensor type strings.
* RunPhaseSubject  -- a generic observer-pattern sink used to signal phase
  changes (training vs evaluation) to subscribed objects.
'''
from typing import Any, List

from markov.sensors.constants import Input
from markov.sensors.composite_sensor import CompositeSensor
from markov.common import ObserverInterface


def construct_sensor(racecar_name, observation_list, factory, model_metadata_info=None):
    '''Build a CompositeSensor from a list of sensor type strings.

    Args:
        racecar_name (str): ROS namespace of the racecar model in Gazebo.
        observation_list (list[str]): Input.value strings for active sensors.
        factory: Object with a create_sensor(racecar_name, sensor_type, config)
            factory method (e.g. SensorFactory).
        model_metadata_info (dict | None): Extra metadata forwarded to sensors
            that need it (e.g. DISCRETIZED_SECTOR_LIDAR).

    Returns:
        CompositeSensor
    '''
    sensor = CompositeSensor()
    if not Input.validate_inputs(observation_list):
        raise Exception('Unsupported input sensor in the observation list')
    for sensor_type in [
        Input.LEFT_CAMERA,
        Input.STEREO,
        Input.CAMERA,
        Input.LIDAR,
        Input.SECTOR_LIDAR,
        Input.DISCRETIZED_SECTOR_LIDAR,
        Input.OBSERVATION,
    ]:
        if sensor_type.value in observation_list:
            config = {'model_metadata': model_metadata_info}
            sensor.add_sensor(factory.create_sensor(racecar_name, sensor_type.value, config))
    return sensor


class RunPhaseSubject(object):
    '''Observable that notifies registered observers when the run phase changes.

    This is a generic observer-pattern implementation with no RL Coach dependency.
    '''

    def __init__(self) -> None:
        self._observer_list_: List[ObserverInterface] = []

    def register(self, observer: ObserverInterface) -> None:
        self._observer_list_.append(observer)

    def unregister(self, observer: ObserverInterface) -> None:
        self._observer_list_.remove(observer)

    def notify(self, data: Any) -> None:
        for observer in self._observer_list_:
            observer.update(data)
