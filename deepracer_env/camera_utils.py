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

"""Utility helpers shared by ROS simulation nodes that manage cameras and
Gazebo model lifecycle.

Exports
-------
WAIT_TO_PREVENT_SPAM : int
    Seconds to sleep between repeated ROS/Gazebo service calls so we do not
    flood the service queue.

wait_for_model(model_name, relative_entity_name)
    Block until the named model is visible in the running Gazebo world.

configure_camera(namespaces)
    Instantiate the follow-car camera for every racecar namespace and a single
    shared top-down camera, then return them as ``(main_cameras, sub_camera)``.
"""

import time
import rospy

from deepracer_env.gazebo_tracker.trackers.get_model_state_tracker import GetModelStateTracker
from deepracer_env.cameras.camera_factory import CameraFactory, CameraType

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

#: Seconds to sleep between retries to avoid spamming service calls.
WAIT_TO_PREVENT_SPAM = 2

# ---------------------------------------------------------------------------
# Functions
# ---------------------------------------------------------------------------


def wait_for_model(model_name, relative_entity_name=''):
    """Block until *model_name* is present in the Gazebo world.

    Args:
        model_name (str): Gazebo model name to wait for.
        relative_entity_name (str): Reference entity forwarded to
            ``get_model_state`` (pass an empty string for the world frame).
    """
    model = GetModelStateTracker.get_instance().get_model_state(model_name,
                                                                relative_entity_name,
                                                                blocking=True,
                                                                auto_sync=False)
    should_wait_for_model = not model.success
    while should_wait_for_model:
        time.sleep(WAIT_TO_PREVENT_SPAM)
        model = GetModelStateTracker.get_instance().get_model_state(model_name,
                                                                    relative_entity_name,
                                                                    blocking=True,
                                                                    auto_sync=False)
        should_wait_for_model = not model.success


def configure_camera(namespaces):
    """Instantiate and register cameras for all racecar namespaces.

    Creates one :class:`~deepracer_env.cameras.handlers.FollowCarCamera` per
    namespace and a single shared
    :class:`~deepracer_env.cameras.handlers.TopCamera`.  Both camera types
    register themselves with the
    :class:`~deepracer_env.cameras.camera_manager.CameraManager` singleton on
    construction.

    Args:
        namespaces (list[str]): Ordered list of racecar ROS namespaces /
            Gazebo model names (e.g. ``["racecar"]`` or
            ``["racecar_0", "racecar_1"]``).

    Returns:
        tuple:
            - **main_cameras** (*dict[str, FollowCarCamera]*): Mapping from
              namespace string to the corresponding follow-car camera instance.
            - **sub_camera** (*TopCamera*): A single shared top-down camera
              that covers the whole track.
    """
    main_cameras = {}
    for namespace in namespaces:
        camera = CameraFactory.create_instance(
            CameraType.FOLLOW_CAR_CAMERA,
            namespace=namespace,
        )
        main_cameras[namespace] = camera

    sub_camera = CameraFactory.create_instance(CameraType.TOP_CAMERA)

    return main_cameras, sub_camera
