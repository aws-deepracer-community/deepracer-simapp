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
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty, EmptyRequest
from markov.track_geom.track_data import FiniteDifference, TrackData
from markov.rospy_wrappers import ServiceProxyWrapper
from markov.camera_utils import configure_camera
from markov import utils

# Amount of time (in seconds) to wait, in order to prevent model state from
# spamming logs while the model is loading
WAIT_TO_PREVENT_SPAM = 2

logger = utils.Logger(__name__, logging.INFO).get_logger()

# The fps of the camera attached to the top camera model

class DeepRacer(object):
    def __init__(self, racecar_names):
        ''' Constructor for the Deep Racer object, will load track and waypoints
        '''
        # Wait for required services to be available
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/pause_physics')
        # We have no guarantees as to when gazebo will load the model, therefore we need
        # to wait until the model is loaded prior to resetting it for the first time
        self.get_model_client = ServiceProxyWrapper('/gazebo/get_model_state', GetModelState)
        for racecar in racecar_names:
            wait_for_model = True
            while wait_for_model:
                # If the model is not loaded the get model service will log an error
                # Therefore, we add an timer to prevent the log from getting spammed with
                # errors
                time.sleep(WAIT_TO_PREVENT_SPAM)
                model = self.get_model_client(racecar, '')
                wait_for_model = not model.success
        # Gazebo service that allows us to position the car
        self.model_state_client = ServiceProxyWrapper('/gazebo/set_model_state', SetModelState)
        # Place the car at the starting point facing the forward direction
        # Instantiate cameras
        main_camera, sub_camera = configure_camera(namespaces=racecar_names)
        # Get the root directory of the ros package, this will contain the models
        deepracer_path = rospkg.RosPack().get_path("deepracer_simulation_environment")
        # Grab the track data
        self.track_data = TrackData.get_instance()
        for racecar in racecar_names:
            car_model_states = self.reset_car(racecar)
            main_camera[racecar].spawn_model(car_model_states,
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

    def reset_car(self, racecar_name):
        ''' Reset's the car on the track
        '''
        # Compute the starting position and heading
        car_model_pose = self.track_data.center_line.interpolate_pose(
            distance=0.0,
            normalized=True,
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
        # Let KVS collect a few frames before pausing the physics, so the car
        # will appear on the track
        time.sleep(1)
        pause_physics = ServiceProxyWrapper('/gazebo/pause_physics', Empty)
        logger.info("Pausing physics after reset")
        pause_physics(EmptyRequest())
        return car_model_state

if __name__ == '__main__':
    rospy.init_node('car_reset_node', anonymous=True)
    # comma seperated racecar names passed as an argument to the node
    RACER_NAMES = sys.argv[1]
    DeepRacer(RACER_NAMES.split(','))
    rospy.spin()
