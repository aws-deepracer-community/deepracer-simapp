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

from markov.defaults import DEFAULT_SUB_CAMERA
from markov.virtual_event.cameras.virtual_event_abs_camera_model import VirtualEventAbsCameraModel


class VirtualEventTopCameraModel(VirtualEventAbsCameraModel):
    """
    VirtualEventTopCameraModel class
    """
    def __init__(self):
        """
        VirtualEventTopCameraModel constructor
        """
        super().__init__(
            camera_type=DEFAULT_SUB_CAMERA,
            model_name="/{}".format("sub_camera"),
            namespace=DEFAULT_SUB_CAMERA)

    def spawn(self):
        """
        Spawn cameras
        """
        self._camera.spawn_model(None, os.path.join(self._deepracer_path, "models",
                                                    "top_camera", "model.sdf"))
