#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

'''This module will launch a ROS node that will have services for retrieving the way
   points and resetting the car. It should serve as an interface into gazebo related
   operations required at start up
'''
import sys
import os
import time
import rclpy
from rclpy.node import Node
import logging
from ament_index_python.packages import get_package_share_directory
from deepracer_msgs.srv import SetModelStates
from deepracer_msgs.msg import ModelState
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty
from markov.track_geom.track_data import FiniteDifference, TrackData
from markov.track_geom.utils import get_start_positions
from markov.rclpy_wrappers import ServiceProxyWrapper
from markov.camera_utils import (wait_for_model, WAIT_TO_PREVENT_SPAM, configure_camera)
import markov.rollout_constants as const
from markov import utils
from markov.world_config import WorldConfig
from markov.utils import force_list, str2bool
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import SIMAPP_CAR_NODE_EXCEPTION, SIMAPP_EVENT_ERROR_CODE_500

from markov.domain_randomizations.constants import GazeboServiceName
from markov.track_geom.constants import SET_MODEL_STATE, START_POS_OFFSET, MIN_START_POS_OFFSET, MAX_START_POS_OFFSET
from deepracer_msgs.srv import GetModelStates
from deepracer_msgs.srv import (GetVisualNames,
                                GetVisuals,
                                SetVisualColors,
                                SetVisualTransparencies,
                                SetVisualVisibles)
from markov.gazebo_utils.model_updater import ModelUpdater

logger = Logger(__name__, logging.INFO).get_logger()


class DeepRacerNode(Node):
    def __init__(self, racecar_names):
        super().__init__('car_reset_node')
        
        ''' Constructor for the Deep Racer object, will load track and waypoints
        '''
        # Wait for required services to be available
        self.wait_for_service('/deepracer/set_model_states', SetModelStates)
        self.wait_for_service('/deepracer/pause_physics_dr', Empty)
        self.wait_for_service('/deepracer/get_model_states', GetModelStates)
        self.wait_for_service('/deepracer/get_visual_names', GetVisualNames)
        self.wait_for_service('/deepracer/get_visuals', GetVisuals)
        self.wait_for_service('/deepracer/set_visual_colors', SetVisualColors)
        self.wait_for_service('/deepracer/set_visual_transparencies', SetVisualTransparencies)
        self.wait_for_service('/deepracer/set_visual_visibles', SetVisualVisibles)

        self.racer_num = len(racecar_names)
        
        # Wait for car spawn delay (racecar_control_kinematics.launch.py has 30s TimerAction before spawning controllers)
        time.sleep(30)
        
        for racecar_name in racecar_names:
            wait_for_model(model_name=racecar_name, relative_entity_name='')
            
        # Get parameters using ROS 2 parameter system
        # car_color is passed from launch file as comma-separated string
        self.declare_parameter('car_color', const.CarColorType.BLACK.value)
        self.declare_parameter('body_shell_type', [const.BodyShellType.DEFAULT.value] * len(racecar_names))
        self.declare_parameter('start_pos_offset', START_POS_OFFSET)
        
        # car_color comes as comma-separated string from launch
        car_color_param = self.get_parameter('car_color').value
        if isinstance(car_color_param, str):
            self.car_colors = [c.strip() for c in car_color_param.split(',')]
        else:
            self.car_colors = force_list(car_color_param)
        
        self.shell_types = force_list(self.get_parameter('body_shell_type').value)
        start_pos_offset = max(min(float(self.get_parameter('start_pos_offset').value), MAX_START_POS_OFFSET),
                               MIN_START_POS_OFFSET)
        
        # Create service clients for Gazebo operations
        self.model_state_client = self.create_client(SetModelStates, '/deepracer/set_model_states')
        self.get_model_states_client = self.create_client(GetModelStates, '/deepracer/get_model_states')
        self.get_visual_names_client = self.create_client(GetVisualNames, '/deepracer/get_visual_names')
        self.get_visuals_client = self.create_client(GetVisuals, '/deepracer/get_visuals')
        self.set_visual_colors_client = self.create_client(SetVisualColors, '/deepracer/set_visual_colors')
        self.set_visual_transparencies_client = self.create_client(SetVisualTransparencies, '/deepracer/set_visual_transparencies')
        self.set_visual_visibles_client = self.create_client(SetVisualVisibles, '/deepracer/set_visual_visibles')

        # Place the car at the starting point facing the forward direction
        # Instantiate cameras
        camera_main_enable = str2bool(WorldConfig.get_param("CAMERA_MAIN_ENABLE", "True"))
        camera_sub_enable = str2bool(WorldConfig.get_param("CAMERA_SUB_ENABLE", "True"))

        if camera_main_enable or camera_sub_enable:
            main_cameras, sub_camera = configure_camera(namespaces=racecar_names)
            [camera.detach() for camera in main_cameras.values()]
            sub_camera.detach()

        # Get the root directory of the ros package, this will contain the models
        deepracer_path = get_package_share_directory("deepracer_simulation_environment")
        
        # Initialize track data and model updater
        self.track_data = TrackData.get_instance()
        # ROS2 FIX: ModelUpdater is a singleton, use get_instance() instead of constructor
        self.model_updater = ModelUpdater.get_instance()
        
        # Set up the car positions (like ROS1)
        start_positions = get_start_positions(len(racecar_names), start_pos_offset)
        
        # Calculate car poses for camera positioning (like ROS1)
        car_poses = []
        for racecar_idx, racecar_name in enumerate(racecar_names):
            # Get car initial start pose
            car_model_pose = self.track_data.get_racecar_start_pose(
                racecar_idx=racecar_idx,
                racer_num=len(racecar_names),
                start_position=start_positions[racecar_idx])
            car_poses.append(car_model_pose)
        
        # Apply car color (ported from ROS1)
        for racecar_idx, racecar_name in enumerate(racecar_names):
            visuals = self.model_updater.get_model_visuals(racecar_name)
            if const.F1 in self.shell_types[racecar_idx]:
                self.model_updater.hide_visuals(
                    visuals=visuals,
                    ignore_keywords=["f1_body_link"] if "with_wheel" in self.shell_types[racecar_idx].lower()
                    else ["wheel", "f1_body_link"])
            else:
                self.model_updater.update_color(visuals, self.car_colors[racecar_idx])

        # Spawn main cameras for each racecar (like ROS1)
        if camera_main_enable:
            for racecar_name, car_pose in zip(racecar_names, car_poses):
                logger.debug("DEBUG: car_pose = %s", car_pose)
                sdf_path = os.path.join(deepracer_path, "models", "camera", "model.sdf")
                logger.info(f"Spawning main camera for {racecar_name}")
                main_cameras[racecar_name].spawn_model(car_pose, sdf_path)
        
        if camera_sub_enable:
            sub_sdf_path = os.path.join(deepracer_path, "models", "top_camera", "model.sdf")
            logger.info("Spawning sub camera model")
            # Spawn the top camera model
            sub_camera.spawn_model(None, sub_sdf_path)
        
        logger.info("DeepRacerNode initialization complete")

    def wait_for_service(self, service_name, service_type, timeout_sec=10.0):
        """Wait for a service to become available"""
        client = self.create_client(service_type, service_name)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f'Service {service_name} not available after waiting {timeout_sec} seconds')
            raise RuntimeError(f'Service {service_name} not available')
        client.destroy()


def main():
    if not rclpy.ok():
        rclpy.init()
    RACER_NUM = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    racecar_names = utils.get_racecar_names(RACER_NUM)
    node = DeepRacerNode(racecar_names)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
