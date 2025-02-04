#!/usr/bin/env python3
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

'''This module will launch a ROS node that will have services for retrieving the way
   points and resetting the car. It should serve as an interface into gazebo related
   operations required at start up
'''
import sys
import os
import time
import rospy
import rospkg
import logging
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty, EmptyRequest
from markov.track_geom.track_data import FiniteDifference, TrackData
from markov.track_geom.utils import get_start_positions
from markov.rospy_wrappers import ServiceProxyWrapper
from markov.camera_utils import (wait_for_model, WAIT_TO_PREVENT_SPAM, configure_camera)
import markov.rollout_constants as const
from markov import utils
from markov.utils import force_list, str2bool
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import SIMAPP_CAR_NODE_EXCEPTION, SIMAPP_EVENT_ERROR_CODE_500

from markov.domain_randomizations.constants import GazeboServiceName
from markov.track_geom.constants import SET_MODEL_STATE, START_POS_OFFSET, MIN_START_POS_OFFSET, MAX_START_POS_OFFSET
from gazebo_msgs.srv import GetModelProperties, GetModelPropertiesRequest
from deepracer_msgs.srv import (GetVisualNames, GetVisualNamesRequest,
                                GetVisuals, GetVisualsRequest, GetVisualsResponse,
                                SetVisualColors, SetVisualColorsRequest, SetVisualColorsResponse,
                                SetVisualTransparencies, SetVisualTransparenciesRequest, SetVisualTransparenciesResponse,
                                SetVisualVisibles, SetVisualVisiblesRequest, SetVisualVisiblesResponse)
from markov.gazebo_utils.model_updater import ModelUpdater

logger = Logger(__name__, logging.INFO).get_logger()


# The fps of the camera attached to the top camera model
class DeepRacer(object):
    def __init__(self, racecar_names):
        ''' Constructor for the Deep Racer object, will load track and waypoints
        '''
        # Wait for required services to be available
        rospy.wait_for_service(SET_MODEL_STATE)
        rospy.wait_for_service(GazeboServiceName.PAUSE_PHYSICS.value)
        rospy.wait_for_service(GazeboServiceName.GET_MODEL_PROPERTIES.value)
        rospy.wait_for_service(GazeboServiceName.GET_VISUAL_NAMES.value)
        rospy.wait_for_service(GazeboServiceName.GET_VISUALS.value)
        rospy.wait_for_service(GazeboServiceName.SET_VISUAL_COLORS.value)
        rospy.wait_for_service(GazeboServiceName.SET_VISUAL_TRANSPARENCIES.value)
        rospy.wait_for_service(GazeboServiceName.SET_VISUAL_VISIBLES.value)

        self.racer_num = len(racecar_names)
        for racecar_name in racecar_names:
            wait_for_model(model_name=racecar_name, relative_entity_name='')
        self.car_colors = force_list(rospy.get_param(const.YamlKey.CAR_COLOR.value,
                                                     [const.CarColorType.BLACK.value] * len(racecar_names)))
        self.shell_types = force_list(rospy.get_param(const.YamlKey.BODY_SHELL_TYPE.value,
                                                      [const.BodyShellType.DEFAULT.value] * len(racecar_names)))
        start_pos_offset = max(min(float(rospy.get_param("START_POS_OFFSET", START_POS_OFFSET)), MAX_START_POS_OFFSET),
                               MIN_START_POS_OFFSET)
        # Gazebo service that allows us to position the car
        self.model_state_client = ServiceProxyWrapper(SET_MODEL_STATE, SetModelState, persistent=True)

        self.get_model_prop = ServiceProxyWrapper(GazeboServiceName.GET_MODEL_PROPERTIES.value,
                                                  GetModelProperties)
        self.get_visual_names = ServiceProxyWrapper(GazeboServiceName.GET_VISUAL_NAMES.value,
                                                    GetVisualNames)
        self.get_visuals = ServiceProxyWrapper(GazeboServiceName.GET_VISUALS.value, GetVisuals)
        self.set_visual_colors = ServiceProxyWrapper(GazeboServiceName.SET_VISUAL_COLORS.value,
                                                     SetVisualColors)
        self.set_visual_transparencies = ServiceProxyWrapper(GazeboServiceName.SET_VISUAL_TRANSPARENCIES.value,
                                                             SetVisualTransparencies)
        self.set_visual_visibles = ServiceProxyWrapper(GazeboServiceName.SET_VISUAL_VISIBLES.value,
                                                       SetVisualVisibles)

        # Place the car at the starting point facing the forward direction
        # Instantiate cameras
        camera_main_enable = str2bool(rospy.get_param("CAMERA_MAIN_ENABLE", "True"))
        camera_sub_enable = str2bool(rospy.get_param("CAMERA_SUB_ENABLE", "True"))

        if camera_main_enable or camera_sub_enable:
            main_cameras, sub_camera = configure_camera(namespaces=racecar_names)
            [camera.detach() for camera in main_cameras.values()]
            sub_camera.detach()

        # Get the root directory of the ros package, this will contain the models
        deepracer_path = rospkg.RosPack().get_path("deepracer_simulation_environment")
        # Grab the track data
        self.track_data = TrackData.get_instance()
        # Set all racers start position in track data
        self.start_positions = get_start_positions(self.racer_num, start_pos_offset)
        # get model update instance
        self.model_update = ModelUpdater.get_instance()
        car_poses = []
        for racecar_idx, racecar_name in enumerate(racecar_names):
            # get car initial start pose
            car_model_pose = self.track_data.get_racecar_start_pose(
                racecar_idx=racecar_idx,
                racer_num=self.racer_num,
                start_position=self.start_positions[racecar_idx])
            # set car at start position
            self.model_update.set_model_pose(model_name=racecar_name,
                                             model_pose=car_model_pose)
            car_poses.append(car_model_pose)
            visuals = self.model_update.get_model_visuals(racecar_name)
            if const.F1 in self.shell_types[racecar_idx]:
                self.model_update.hide_visuals(
                    visuals=visuals,
                    ignore_keywords=["f1_body_link"] if "with_wheel" in self.shell_types[racecar_idx].lower()
                    else ["wheel", "f1_body_link"])
            else:
                self.model_update.update_color(visuals, self.car_colors[racecar_idx])

        # Let KVS collect a few frames before pausing the physics, so the car
        # will appear on the track
        time.sleep(1)
        pause_physics = ServiceProxyWrapper('/gazebo/pause_physics', Empty)
        logger.info("Pausing physics after initializing the cars")
        pause_physics(EmptyRequest())

        if camera_main_enable:
            for racecar_name, car_pose in zip(racecar_names, car_poses):
                main_cameras[racecar_name].spawn_model(car_pose,
                                                    os.path.join(deepracer_path, "models",
                                                                    "camera", "model.sdf"))

        if camera_sub_enable:
            logger.info("Spawning sub camera model")
            # Spawn the top camera model
            sub_camera.spawn_model(None, os.path.join(deepracer_path, "models",
                                                    "top_camera", "model.sdf"))

if __name__ == '__main__':
    rospy.init_node('car_reset_node', anonymous=True)
    try:
        # comma separated racecar names passed as an argument to the node
        RACER_NUM = int(sys.argv[1])
        racecar_names = utils.get_racecar_names(RACER_NUM)
        DeepRacer(racecar_names)
    except Exception as ex:
        log_and_exit("Exception in car node: {}".format(ex),
                     SIMAPP_CAR_NODE_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)
    rospy.spin()
