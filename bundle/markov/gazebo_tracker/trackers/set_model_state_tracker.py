# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

import logging
import threading
import traceback
from markov.log_handler.deepracer_exceptions import GenericRolloutException
from markov.log_handler.logger import Logger
import copy

from deepracer_msgs.msg import ModelState
from deepracer_msgs.srv import SetModelStates
from markov.track_geom.constants import SET_MODEL_STATES
from markov.rclpy_wrappers import ServiceProxyWrapper, ROS2NodeManager
from markov.gazebo_tracker.abs_tracker import AbstractTracker
from markov.gazebo_tracker.trackers.get_model_state_tracker import GetModelStateTracker
import markov.gazebo_tracker.constants as consts
from markov.rclpy_constants import DEFAULT_ROS2_NODE_NAME

logger = Logger(__name__, logging.INFO).get_logger()


class SetModelStateTracker(AbstractTracker):
    """
    SetModelState Tracker class
    """
    _instance_ = None

    @staticmethod
    def get_instance():
        """Method for getting a reference to the SetModelState Tracker object"""
        if SetModelStateTracker._instance_ is None:
            SetModelStateTracker()
        return SetModelStateTracker._instance_

    def __init__(self):
        if SetModelStateTracker._instance_ is not None:
            raise GenericRolloutException("Attempting to construct multiple SetModelState Tracker")

        self.lock = threading.RLock()
        # Use dictionary like ROS1 instead of list to prevent duplicate model states, contributing to SetModelState flood
        self.model_state_map = {}

        # ROS1-like: explicit service waiting first, then create ServiceProxyWrapper  
        node = ROS2NodeManager.get_instance(DEFAULT_ROS2_NODE_NAME).get_node()
        client = node.create_client(SetModelStates, SET_MODEL_STATES)
        
        # Equivalent of rospy.wait_for_service() - wait indefinitely
        while not client.wait_for_service(timeout_sec=1.0):
            pass  # Keep waiting like ROS1 did
            
        # Now create simple ServiceProxyWrapper (should work immediately)
        self.set_model_states = ServiceProxyWrapper(SET_MODEL_STATES, SetModelStates)

        SetModelStateTracker._instance_ = self
        super(SetModelStateTracker, self).__init__(priority=consts.TrackerPriority.LOW)

    def set_model_state(self, model_state, blocking=False):
        """
        Set the model state based on the model_state passed
        Args:
            model_state (ModelState): model state object
            blocking (bool): flag to block or not
        Returns:
            response msg (bool): The set model state response msg
        """
        
        logger.debug("Setting model state position: x=%.3f, y=%.3f, z=%.3f", 
                   model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z)
        
        # DEBUG: Add call stack trace to identify source of wrong position calls
        # Target the remaining wrong position we're seeing: x≈0.144, y≈0.000
        if (abs(model_state.pose.position.x - 0.144) < 0.01 and 
            abs(model_state.pose.position.y - 0.000) < 0.01):
            stack_lines = traceback.format_stack()
            for i, line in enumerate(stack_lines):
                logger.debug(f"Stack trace line {i}: {line.strip()}")
        
        with self.lock:
            if blocking:
                # Create a request with the model states
                request = SetModelStates.Request()
                request.model_states = [model_state]
                
                # Call the service
                try:
                    response = self.set_model_states(request)
                    if hasattr(response, 'status') and len(response.status) > 0:
                        # Match ROS1 logic: check both success AND status
                        final_success = response.success and response.status[0]
                    else:
                        final_success = response.success
                    
                    if hasattr(response, 'status_message'):
                        logger.debug(f"Set model state status: {response.status_message}")
                    elif hasattr(response, 'messages') and len(response.messages) > 0:
                        logger.debug(f"Set model state messages: {response.messages}")
                    
                    # Proper ROS2 synchronization - invalidate GetModelState cache instead of time delay
                    if final_success:
                        # Clear GetModelState cache to force fresh read from Gazebo
                        get_tracker = GetModelStateTracker.get_instance()
                        with get_tracker.lock:
                            # Clear cache entries for this model (handle both key formats)
                            model_name = model_state.model_name
                            keys_to_remove = []
                            for key in get_tracker.model_map.keys():
                                if isinstance(key, tuple) and key[0] == model_name:
                                    keys_to_remove.append(key)
                                elif isinstance(key, str) and key == model_name:
                                    keys_to_remove.append(key)
                            
                            for key in keys_to_remove:
                                del get_tracker.model_map[key]
                        
                    
                    return final_success
                except Exception as e:
                    logger.error(f"Error in set_model_state: {e}")
                    return False
            else:
                # Use dictionary like ROS1 to prevent duplicates
                # If same model gets multiple set_model_state calls, only keep the latest
                self.model_state_map[model_state.model_name] = copy.deepcopy(model_state)
                return True

    def update_tracker(self, delta_time, sim_time):
        """
        Update the tracker - required abstract method implementation
        Args:
            delta_time (float): time elapsed since last update
            sim_time (float): current simulation time
        """
        
        with self.lock:
            # Use dictionary like ROS1 to prevent duplicate commands
            if self.model_state_map:
                # Create a request with all pending model states (no duplicates)
                request = SetModelStates.Request()
                request.model_states = list(self.model_state_map.values())
                
                
                # Call the service
                try:
                    response = self.set_model_states(request)
                except Exception as e:
                    logger.error(f"Error setting model states: {e}")
                
                # Clear the dictionary
                self.model_state_map = {}
            else:
                logger.debug("Non-blocking model state update - adding to map")
