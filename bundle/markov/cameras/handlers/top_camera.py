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

import sys
import math
import logging
import xml.etree.ElementTree as ET
import rospy

from deepracer_simulation_environment.srv import TopCamDataSrvResponse, TopCamDataSrv
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from markov.track_geom.track_data import TrackData
from markov.track_geom.utils import euler_to_quaternion
from markov.cameras.abs_camera import AbstractCamera
from markov.cameras.constants import CameraSettings, PYTHON_2
from markov.log_handler.logger import Logger
from markov.gazebo_tracker.trackers.set_model_state_tracker import SetModelStateTracker

# Height value is determined from AWS track and is maintained to prevent z fighting in top down
# view
CAMERA_HEIGHT = 6.0
# Percentage to pad the image so that the frame boundary is not exactly on the track
PADDING_PCT = 0.25
# The default horizontal field of view
DEFAULT_H_FOV = 1.13
# Default resolution
DEFAULT_RESOLUTION = (640, 480)
# Logger object
LOG = Logger(__name__, logging.INFO).get_logger()


class TopCamera(AbstractCamera):
    """this module is for top camera"""
    name = "top_camera"

    def __init__(self, namespace=None, model_name=None):
        self._python_version = sys.version_info[0]
        super(TopCamera, self).__init__(TopCamera.name, namespace=namespace,
                                        model_name=model_name)
        self.track_data = TrackData.get_instance()
        x_min, y_min, x_max, y_max = self.track_data.outer_border.bounds
        horizontal_width = (x_max - x_min) * (1.0 + PADDING_PCT)
        vertical_width = (y_max - y_min) * (1.0 + PADDING_PCT)
        horizontal_fov = DEFAULT_H_FOV
        try:
            if horizontal_width >= vertical_width:
                horizontal_fov = 2.0 * math.atan(0.5 * horizontal_width / CAMERA_HEIGHT)
            else:
                vertical_fov = math.atan(0.5 * vertical_width / CAMERA_HEIGHT)
                aspect_ratio = float(DEFAULT_RESOLUTION[0]) / float(DEFAULT_RESOLUTION[1])
                horizontal_fov = 2.0 * math.atan(aspect_ratio * math.tan(vertical_fov))
        except Exception as ex:
            LOG.info('Unable to compute top camera fov, using default: %s', ex)

        self.camera_settings_dict = CameraSettings.get_empty_dict()
        self.camera_settings_dict[CameraSettings.HORZ_FOV] = horizontal_fov
        self.camera_settings_dict[CameraSettings.PADDING_PCT] = PADDING_PCT
        self.camera_settings_dict[CameraSettings.IMG_WIDTH] = DEFAULT_RESOLUTION[0]
        self.camera_settings_dict[CameraSettings.IMG_HEIGHT] = DEFAULT_RESOLUTION[1]

        rospy.Service('get_top_cam_data', TopCamDataSrv, self._handle_get_top_cam_data)

    def _handle_get_top_cam_data(self, req):
        '''Response handler for clients requesting the camera settings data
           req - Client request, which should be an empty request
        '''
        return TopCamDataSrvResponse(self.camera_settings_dict[CameraSettings.HORZ_FOV],
                                     self.camera_settings_dict[CameraSettings.PADDING_PCT],
                                     self.camera_settings_dict[CameraSettings.IMG_WIDTH],
                                     self.camera_settings_dict[CameraSettings.IMG_HEIGHT])

    def _get_sdf_string(self, camera_sdf_path):
        """
        Get sdf string
        Virtual event is running in python3 while training, eval, and leaderboard submission
        is running in python2

        In python3
        https://docs.python.org/3.8/library/xml.etree.elementtree.html#xml.etree.ElementTree.ElementTree
        Use encoding="unicode" to generate a Unicode string (otherwise, a bytestring is generated)
        In python2
        https://docs.python.org/2.7/library/xml.etree.elementtree.html#xml.etree.ElementTree.ElementTree
        Use encoding="us-ascii", returns an encoded string containing the XML data

        Args:
            camera_sdf_path (str): camera sdf path

        Returns:
            str: an encoded string containing the XML data
        """        
        tree = ET.parse(camera_sdf_path)
        root = tree.getroot()
        for fov in root.iter('horizontal_fov'):
            fov.text = str(self.camera_settings_dict[CameraSettings.HORZ_FOV])
        encoding = "unicode"
        if self._python_version <= PYTHON_2:
            encoding = "us-ascii"
        return "<?xml version=\"1.0\"?>\n {}".format(ET.tostring(root,
                                                                 encoding=encoding,
                                                                 method='xml'))

    def _get_initial_camera_pose(self, car_pose):
        # get the bounds
        x_min, y_min, x_max, y_max = self.track_data.outer_border.bounds
        # update camera position
        model_pose = Pose()
        model_pose.position.x = (x_min + x_max) / 2.0
        model_pose.position.y = (y_min + y_max) / 2.0
        model_pose.position.z = CAMERA_HEIGHT
        x, y, z, w = euler_to_quaternion(roll=1.57079, pitch=1.57079, yaw=3.14159)
        model_pose.orientation.x = x
        model_pose.orientation.y = y
        model_pose.orientation.z = z
        model_pose.orientation.w = w
        return model_pose

    def _reset(self, car_pose):
        """Reset camera position based on the track size"""
        if self.is_reset_called:
            return
        # update camera position
        model_pose = self._get_initial_camera_pose(car_pose)
        camera_model_state = ModelState()
        camera_model_state.model_name = self.model_name
        camera_model_state.pose = model_pose
        SetModelStateTracker.get_instance().set_model_state(camera_model_state)

    def _update(self, model_state, delta_time):
        pass