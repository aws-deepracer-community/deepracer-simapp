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

import abc
import threading

from markov.log_handler.deepracer_exceptions import GenericRolloutException
from markov.rclpy_wrappers import ServiceProxyWrapper
from markov.track_geom.constants import SPAWN_SDF_MODEL
from markov.cameras.camera_manager import CameraManager
from deepracer_msgs.srv import SpawnModel

# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})


class AbstractCamera(ABC):
    """
    Abstract Camera method
    """
    def __init__(self, name, namespace, model_name):
        if not name or not isinstance(name, str):
            raise GenericRolloutException("Camera name cannot be None or empty string")
        self._name = name
        self._model_name = model_name or self._name
        self._namespace = namespace or 'default'
        self.lock = threading.Lock()
        self.is_reset_called = False
        
        # NOTE: SPAWN_SDF_MODEL may need to be set to '/world/default/create'
        # Use longer timeout for spawn service as it takes time to initialize after model is ready
        self.spawn_sdf_model = ServiceProxyWrapper(SPAWN_SDF_MODEL, SpawnModel, max_retry_attempts=10, timeout_sec=10.0)
        CameraManager.get_instance().add(self, namespace)

    @property
    def name(self):
        """Return name of Camera

        Returns:
            (str): the name of camera
        """
        return self._name

    @property
    def model_name(self):
        """Return name of gazebo topic

        Returns:
            (str): the name of camera
        """
        return self._model_name

    @property
    def namespace(self):
        """Return namespace of camera in camera manager

        Returns:
            (str): the namespace of camera in camera manager
        """
        return self._namespace

    def reset_pose(self, car_pose):
        """
        Reset the camera pose

        Args:
            car_pose (Pose): Pose of car
        """
        with self.lock:
            self._reset(car_pose)
            self.is_reset_called = True

    def spawn_model(self, car_pose, camera_sdf_path):
        """
        Spawns a sdf model located in the given path

        Args:
            car_pose (Pose): Pose of car
            camera_sdf_path (string): full path to the location of sdf file
        """
        camera_sdf = self._get_sdf_string(camera_sdf_path)
        camera_pose = self._get_initial_camera_pose(car_pose)
        req = SpawnModel.Request()
        # ROS2 FIX: Use model_name instead of name for SpawnModel request
        # req.name = self.model_name  # OLD: This doesn't work in ROS2
        req.model_name = self.model_name  # NEW: Correct attribute for ROS2
        req.robot_namespace = self._namespace
        # OLD ROS1→ROS2 migration error: req.initial_post = camera_pose
        # FIXED ROS2: Correct attribute name is initial_pose (from service definition)
        req.initial_pose = camera_pose
        req.model_xml = camera_sdf
        req.reference_frame = ''
        self.spawn_sdf_model(req)

    def update_pose(self, car_pose, delta_time):
        """
        Update the camera pose

        Args:
            car_pose (Pose): Pose of car
            delta_time (float): time delta from last update
        """
        with self.lock:
            if self.is_reset_called:
                self._update(car_pose, delta_time)

    @abc.abstractmethod
    def _get_sdf_string(self, camera_sdf_path):
        """
        Reads the sdf file and converts it to a string in
        memory

        Args:
            camera_sdf_path (string): full path to the location of sdf file
        """
        raise NotImplementedError('Camera must read and convert model sdf file')

    @abc.abstractmethod
    def _reset(self, car_pose):
        """
        Reset the camera pose
        """
        raise NotImplementedError('Camera must be able to reset')

    @abc.abstractmethod
    def _update(self, car_pose, delta_time):
        """Update the camera pose
        """
        raise NotImplementedError('Camera must be able to update')

    @abc.abstractmethod
    def _get_initial_camera_pose(self, car_pose):
        """compuate camera pose
        """
        raise NotImplementedError('Camera must be able to compuate pose')

    def detach(self):
        """Detach camera from manager"""
        CameraManager.get_instance().remove(self, self.namespace)

