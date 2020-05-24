#!/usr/bin/env python3
##############################################################
#                                                            #
#   Copyright 2019 Amazon.com, Inc. or its affiliates.       #
#   All Rights Reserved.                                     #
#                                                            #
##############################################################

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
from markov.track_geom.constants import START_POS_OFFSET
from markov.rospy_wrappers import ServiceProxyWrapper
from markov.camera_utils import (wait_for_model, WAIT_TO_PREVENT_SPAM, configure_camera)
import markov.rollout_constants as const
from markov import utils
from markov.utils import force_list
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import SIMAPP_CAR_NODE_EXCEPTION, SIMAPP_EVENT_ERROR_CODE_500

from markov.domain_randomizations.constants import GazeboServiceName
from markov.track_geom.constants import SET_MODEL_STATE
from gazebo_msgs.srv import GetModelProperties, GetModelPropertiesRequest
from deepracer_msgs.srv import (GetVisualNames, GetVisualNamesRequest,
                                GetVisuals, GetVisualsRequest, GetVisualsResponse,
                                SetVisualColors, SetVisualColorsRequest, SetVisualColorsResponse,
                                SetVisualTransparencies, SetVisualTransparenciesRequest, SetVisualTransparenciesResponse,
                                SetVisualVisibles, SetVisualVisiblesRequest, SetVisualVisiblesResponse)


logger = Logger(__name__, logging.INFO).get_logger()


# The fps of the camera attached to the top camera model
class DeepRacer(object):
    def __init__(self, racecar_names):
        ''' Constructor for the Deep Racer object, will load track and waypoints
        '''
        # Wait for required services to be available
        rospy.wait_for_service(SET_MODEL_STATE)
        rospy.wait_for_service('/gazebo/pause_physics')
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
        # Gazebo service that allows us to position the car
        self.model_state_client = ServiceProxyWrapper(SET_MODEL_STATE, SetModelState)

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
        main_cameras, sub_camera = configure_camera(namespaces=racecar_names)
        [camera.detach() for camera in main_cameras.values()]
        sub_camera.detach()
        # Get the root directory of the ros package, this will contain the models
        deepracer_path = rospkg.RosPack().get_path("deepracer_simulation_environment")
        # Grab the track data
        self.track_data = TrackData.get_instance()
        # Set all racers start position in track data
        self.start_positions = [START_POS_OFFSET * idx for idx in reversed(range(self.racer_num))]
        car_poses = []
        for racecar_idx, racecar_name in enumerate(racecar_names):
            car_model_state = self.get_initial_position(racecar_name,
                                                        racecar_idx)
            car_poses.append(car_model_state.pose)
            self.update_model_visual(racecar_name,
                                     self.shell_types[racecar_idx],
                                     self.car_colors[racecar_idx])

        # Let KVS collect a few frames before pausing the physics, so the car
        # will appear on the track
        time.sleep(1)
        pause_physics = ServiceProxyWrapper('/gazebo/pause_physics', Empty)
        logger.info("Pausing physics after reset")
        pause_physics(EmptyRequest())

        for racecar_name, car_pose in zip(racecar_names, car_poses):
            main_cameras[racecar_name].spawn_model(car_pose,
                                                  os.path.join(deepracer_path, "models",
                                                               "camera", "model.sdf"))

        logger.info("Spawning sub camera model")
        # Spawn the top camera model
        sub_camera.spawn_model(None, os.path.join(deepracer_path, "models",
                                                  "top_camera", "model.sdf"))
        # Let KVS collect a few frames before pausing the physics, so the car
        # will appear on the track
        time.sleep(1)
        pause_physics = ServiceProxyWrapper('/gazebo/pause_physics', Empty)
        logger.info("Pausing physics")
        pause_physics(EmptyRequest())

    def _update_color(self, visuals, car_color):
        link_names = []
        visual_names = []
        ambients, diffuses, speculars, emissives = [], [], [], []

        for visual_name, link_name in zip(visuals.visual_names, visuals.link_names):
            if "car_body_link_v2" in visual_name or "top_cover_link" in visual_name:
                visual_names.append(visual_name)
                link_names.append(link_name)
                ambient = ColorRGBA(const.COLOR_MAP[car_color].r * 0.1,
                                    const.COLOR_MAP[car_color].g * 0.1,
                                    const.COLOR_MAP[car_color].b * 0.1,
                                    const.COLOR_MAP[car_color].a)
                diffuse = ColorRGBA(const.COLOR_MAP[car_color].r * 0.35,
                                    const.COLOR_MAP[car_color].g * 0.35,
                                    const.COLOR_MAP[car_color].b * 0.35,
                                    const.COLOR_MAP[car_color].a)

                ambients.append(ambient)
                diffuses.append(diffuse)
                speculars.append(const.DEFAULT_COLOR)
                emissives.append(const.DEFAULT_COLOR)
        if len(visual_names) > 0:
            req = SetVisualColorsRequest()
            req.visual_names = visual_names
            req.link_names = link_names
            req.ambients = ambients
            req.diffuses = diffuses
            req.speculars = speculars
            req.emissives = emissives
            self.set_visual_colors(req)

    def _hide_visuals(self, visuals):
        link_names = []
        visual_names = []

        for visual_name, link_name in zip(visuals.visual_names, visuals.link_names):
            if "wheel" not in visual_name \
                    and "car_body_link_v2" not in visual_name \
                    and "top_cover_link" not in visual_name \
                    and "car_body_link" not in visual_name:
                visual_names.append(visual_name)
                link_names.append(link_name)

        req = SetVisualTransparenciesRequest()
        req.link_names = link_names
        req.visual_names = visual_names
        req.transparencies = [1.0] * len(link_names)
        self.set_visual_transparencies(req)

        req = SetVisualVisiblesRequest()
        req.link_names = link_names
        req.visual_names = visual_names
        req.visibles = [False] * len(link_names)
        self.set_visual_visibles(req)

    def update_model_visual(self, racecar_name, body_shell_type, car_color):
        # Get all model's link names
        body_names = self.get_model_prop(GetModelPropertiesRequest(model_name=racecar_name)) \
            .body_names
        link_names = ["%s::%s" % (racecar_name, b) for b in body_names]
        res = self.get_visual_names(GetVisualNamesRequest(link_names=link_names))
        get_visuals_req = GetVisualsRequest(link_names=res.link_names,
                                            visual_names=res.visual_names)
        visuals = self.get_visuals(get_visuals_req)

        if const.F1 in body_shell_type.lower():
            self._hide_visuals(visuals=visuals)
        else:
            self._update_color(visuals=visuals,
                               car_color=car_color)

    def get_initial_position(self, racecar_name, racecar_idx):
        ''' get initial car position on the track
        '''
        # Compute the starting position and heading
        # single racer: spawn at centerline
        if self.racer_num == 1:
            car_model_pose = self.track_data.center_line.interpolate_pose(
                distance=0.0,
                normalized=True,
                finite_difference=FiniteDifference.FORWARD_DIFFERENCE)
        # multi racers: spawn odd car at inner lane and even car at outer lane
        else:
            lane = self.track_data.inner_lane if racecar_idx % 2 else \
                self.track_data.outer_lane
            car_model_pose = lane.interpolate_pose(
                distance=self.start_positions[racecar_idx],
                normalized=False,
                finite_difference=FiniteDifference.FORWARD_DIFFERENCE)
        # Construct the model state and send to Gazebo
        car_model_state = ModelState()
        car_model_state.model_name = racecar_name
        car_model_state.pose = car_model_pose
        car_model_state.twist.linear.x = 0
        car_model_state.twist.linear.y = 0
        car_model_state.twist.linear.z = 0
        car_model_state.twist.angular.x = 0
        car_model_state.twist.angular.y = 0
        car_model_state.twist.angular.z = 0
        self.model_state_client(car_model_state)
        return car_model_state


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
