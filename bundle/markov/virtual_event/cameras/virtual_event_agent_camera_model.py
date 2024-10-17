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

import os

from markov.defaults import DEFAULT_MAIN_CAMERA
from markov.virtual_event.cameras.virtual_event_abs_camera_model import VirtualEventAbsCameraModel


class VirtualEventAgentCameraModel(VirtualEventAbsCameraModel):
    """
    VirtualEventCameraManager class
    """
    def __init__(self, camera_namespace, start_pose):
        """
        VirtualEventAgentCameraModel constructor

        Args:
            camera_namespace (str): camera namespace
            start_pose (Pose): camera start pose
        """
        super().__init__(
            camera_type=DEFAULT_MAIN_CAMERA,
            model_name="/{}/{}".format(camera_namespace, "main_camera"),
            namespace=camera_namespace)
        self._start_pose = start_pose

    def spawn(self):
        """
        Spawn cameras
        """
        self._camera.spawn_model(self._start_pose,
                                 os.path.join(self._deepracer_path, "models",
                                              "camera", "model.sdf"))

    def reset_pose(self):
        """
        Reset camera pose
        """
        self._camera.reset_pose(
            car_pose=self._start_pose)
