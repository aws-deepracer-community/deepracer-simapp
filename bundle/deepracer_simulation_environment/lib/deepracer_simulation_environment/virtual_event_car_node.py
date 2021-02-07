#!/usr/bin/env python
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
import math
import logging
import rospy
import rospkg
from markov.track_geom.track_data import FiniteDifference, TrackData
from markov.track_geom.utils import get_hide_positions, get_start_positions
from markov.camera_utils import (wait_for_model, configure_camera)
import markov.rollout_constants as const
from markov import utils
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import SIMAPP_CAR_NODE_EXCEPTION, SIMAPP_EVENT_ERROR_CODE_500
from markov.gazebo_utils.model_updater import ModelUpdater
from markov.virtual_event.constants import VIRTUAL_EVENT
from markov.cameras.camera_manager import CameraManager

LOG = Logger(__name__, logging.INFO).get_logger()


class DeepRacer(object):
    def __init__(self, racecar_names):
        ''' Constructor for the Deep Racer object, will load track and waypoints
        '''
        self._model_updater = ModelUpdater.get_instance()
        self.racer_num = len(racecar_names)
        for racecar_name in racecar_names:
            wait_for_model(model_name=racecar_name, relative_entity_name='')
        self.car_colors = [const.CarColorType.BLACK.value] * len(racecar_names)
        shell_type = rospy.get_param(const.YamlKey.BODY_SHELL_TYPE.value,
                                     const.BodyShellType.DEFAULT.value)
        self.shell_types = [shell_type] * len(racecar_names)
        # Grab the track data
        self.track_data = TrackData.get_instance()
        # Set all cars at the park position in track data
        self._park_positions = get_hide_positions(self.racer_num)
        yaw = 0.0 if self.track_data.is_ccw else math.pi
        for racecar_idx, racecar_name in enumerate(racecar_names):
            car_model_state = self._model_updater.set_model_position(racecar_name,
                                                                     self._park_positions[racecar_idx],
                                                                     yaw)
            visuals = self._model_updater.get_model_visuals(racecar_name)
            if const.F1 in self.shell_types[racecar_idx]:
                self._model_updater.hide_visuals(visuals=visuals)
            else:
                self._model_updater.update_color(visuals, self.car_colors[racecar_idx])
        LOG.info("[virtual event manager] Init RaceCars complete")
        # spawn follow car and top down camera
        self._spawn_cameras()
        # Pause physics after initialization
        self._model_updater.pause_physics()

    def _spawn_cameras(self):
        '''helper method for initializing cameras
        '''
        # setting up virtual event top and follow camera in CameraManager
        # virtual event configure camera does not need to wait for car to spawm because
        # follow car camera is not tracking any car initially
        main_cameras, sub_camera = configure_camera(namespaces=[VIRTUAL_EVENT],
                                                    is_wait_for_model=False)
        camera_manager = CameraManager.get_instance()

        # pop all camera under virtual event namespace
        camera_manager.pop(namespace=VIRTUAL_EVENT)

        # Get the root directory of the ros package, this will contain the models
        deepracer_path = rospkg.RosPack().get_path("deepracer_simulation_environment")

        # Spawn the follow car camera
        LOG.info("[virtual event manager] Spawning virtual event follow car camera model")
        initial_pose = self.track_data.get_racecar_start_pose(
            racecar_idx=0,
            racer_num=1,
            start_position=get_start_positions(1)[0])
        main_cameras[VIRTUAL_EVENT].spawn_model(initial_pose,
                                                os.path.join(deepracer_path, "models",
                                                             "camera", "model.sdf"))

        LOG.info("[virtual event manager] Spawning sub camera model")
        # Spawn the top camera model
        sub_camera.spawn_model(None, os.path.join(deepracer_path, "models",
                                                  "top_camera", "model.sdf"))


if __name__ == '__main__':
    rospy.init_node('virtual_event_car_node', anonymous=True)
    try:
        # comma separated racecar names passed as an argument to the node
        RACER_NUM = int(sys.argv[1])
        RACECAR_NAMES = utils.get_racecar_names(RACER_NUM)
        DeepRacer(RACECAR_NAMES)
    except Exception as ex:
        log_and_exit("Exception in car node: {}".format(ex),
                     SIMAPP_CAR_NODE_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)
    rospy.spin()
