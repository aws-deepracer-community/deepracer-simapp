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
import rospkg

from markov.cameras.camera_factory import CameraFactory
from markov.cameras.camera_manager import CameraManager
from markov.spawn.constants import DeepRacerPackages

# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})


class VirtualEventAbsCameraModel(ABC):
    """
    VirtualEventAbsCameraModel class
    """
    def __init__(self, camera_type, model_name, namespace):
        """
        VirtualEventAbsCameraModel constructor

        Args:
            camera_type (str): camera type
            model_name (str): camera model name
            namespace (str): camera namespace
        """
        self._camera_manager = CameraManager.get_instance()
        self._namespace = namespace
        self._deepracer_path = rospkg.RosPack().get_path(
            DeepRacerPackages.DEEPRACER_SIMULATION_ENVIRONMENT)
        self._camera = CameraFactory.create_instance(
            camera_type=camera_type,
            model_name=model_name,
            namespace=namespace)
        self.detach()

    @abc.abstractmethod
    def spawn(self):
        """
        Spawn camera
        """
        raise NotImplementedError('VirtualEventAbsCameraModel must implement spawn')

    def detach(self):
        """
        stop following item specifying by namespace through popping camera name
        """
        self._camera_manager.pop(self._namespace)

    def attach(self):
        """
        start following item specifying by namespace through adding camera name
        """
        self._camera_manager.add(self._camera,
                                 self._namespace)
