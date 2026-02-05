# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

import threading
from markov.log_handler.deepracer_exceptions import GenericRolloutException
import copy
import logging

from deepracer_msgs.srv import GetModelStates
from markov.track_geom.constants import GET_MODEL_STATES
from markov.rclpy_wrappers import ServiceProxyWrapper
from markov.gazebo_tracker.abs_tracker import AbstractTracker
import markov.gazebo_tracker.constants as consts
from markov.rclpy_constants import DEFAULT_ROS2_NODE_NAME


class GetModelStateResponse:
    def __init__(self):
        self.pose = None
        self.twist = None
        self.success = False
        self.status_message = ""


class GetModelStateTracker(AbstractTracker):
    """
    GetModelState Tracker class
    """
    _instance_ = None

    @staticmethod
    def get_instance():
        """Method for getting a reference to the GetModelState Tracker object"""
        if GetModelStateTracker._instance_ is None:
            GetModelStateTracker()
        return GetModelStateTracker._instance_

    def __init__(self):
        if GetModelStateTracker._instance_ is not None:
            raise GenericRolloutException("Attempting to construct multiple GetModelState Tracker")

        self.lock = threading.RLock()
        self.model_map = {}
        self.model_names = []
        self.relative_entity_names = []

        # ROS1-like: explicit service waiting first, then create ServiceProxyWrapper
        from markov.rclpy_wrappers import ROS2NodeManager
        node = ROS2NodeManager.get_instance(DEFAULT_ROS2_NODE_NAME).get_node()
        client = node.create_client(GetModelStates, GET_MODEL_STATES)
        
        # Equivalent of rospy.wait_for_service() - wait indefinitely
        while not client.wait_for_service(timeout_sec=1.0):
            pass  # Keep waiting like ROS1 did
            
        # Now create simple ServiceProxyWrapper (should work immediately)
        self.get_model_states = ServiceProxyWrapper(GET_MODEL_STATES, GetModelStates)

        GetModelStateTracker._instance_ = self
        super(GetModelStateTracker, self).__init__(priority=consts.TrackerPriority.HIGH)

    def get_model_state(self, model_name, relative_entity_name='', blocking=False):
        with self.lock:
            # Use consistent cache key format like ROS1 (model_name, relative_entity_name)
            cache_key = (model_name, relative_entity_name)
            
            if model_name not in self.model_names:
                self.model_names.append(model_name)
            if relative_entity_name not in self.relative_entity_names:
                self.relative_entity_names.append(relative_entity_name)
                
            if blocking or cache_key not in self.model_map:
                # Create a request with the model names and relative entity names
                request = GetModelStates.Request()
                request.model_names = self.model_names
                request.relative_entity_names = self.relative_entity_names
                try:
                    # Call the service
                    response = self.get_model_states(request)
                    
                    # Add detailed logging of the raw service response
                    from markov.log_handler.logger import Logger
                    logger = Logger(__name__, logging.ERROR).get_logger()
                    
                    # Log the actual position returned by GetModelState
                    if response.success and response.model_states:
                        for model_state in response.model_states:
                            if model_state.model_name == model_name:
                                pos = model_state.pose.position
                    
                    # Process the response
                    if not response.success:
                        msg = GetModelStateResponse()
                        msg.success = False
                        msg.status_message = response.status_message
                        return msg
                    for i, state in enumerate(response.model_states):
                        
                        if state.model_name == model_name:
                            model_state = GetModelStateResponse()
                            model_state.pose = state.pose
                            model_state.twist = state.twist
                            # Use per-model status (1=found, 0=not found), not overall response.success
                            model_state.success = bool(response.status[i]) if i < len(response.status) else False
                            model_state.status_message = response.messages[i] if i < len(response.messages) else ""
                            # Update cache with consistent key format
                            self.model_map[cache_key] = model_state
                            return model_state
                    
                    # If model not found in response
                    msg = GetModelStateResponse()
                    msg.success = False
                    msg.status_message = f"Model {model_name} not found"
                    return msg
                except Exception as ex:
                    # Log the error
                    from markov.log_handler.logger import Logger
                    logger = Logger(__name__, logging.ERROR).get_logger()
                    logger.error(f"Error getting model state for {model_name}: {ex}")
                    msg = GetModelStateResponse()
                    msg.success = False
                    msg.status_message = str(ex)
                    return msg
            else:
                # Return cached result using consistent key
                if cache_key in self.model_map:
                    return copy.deepcopy(self.model_map[cache_key])
                msg = GetModelStateResponse()
                msg.success = False
                msg.status_message = f"Model {model_name} not cached"
                return msg

    def update_tracker(self, delta_time, sim_time):
        """
        Update model_states of the models that this tracker is tracking

        Args:
            delta_time (float): delta time
            sim_time (float): simulation time
        """
        
        if self.model_names:
            with self.lock:
                try:
                    request = GetModelStates.Request()
                    request.model_names = self.model_names
                    request.relative_entity_names = self.relative_entity_names
                    
                    response = self.get_model_states(request)
                    # Check for None response (can happen during shutdown)
                    if response is None:
                        return
                    if response.success:
                        for model_state, status in zip(response.model_states, response.status):
                            if status:
                                model_name = model_state.model_name
                                # Find corresponding relative_entity_name for this model
                                model_index = self.model_names.index(model_name) if model_name in self.model_names else 0
                                relative_entity_name = self.relative_entity_names[model_index] if model_index < len(self.relative_entity_names) else ""
                                cache_key = (model_name, relative_entity_name)
                                
                                # Create GetModelStateResponse object for consistency
                                cached_state = GetModelStateResponse()
                                cached_state.pose = model_state.pose
                                cached_state.twist = model_state.twist
                                cached_state.success = True
                                cached_state.status_message = ""
                                self.model_map[cache_key] = cached_state
                except Exception as ex:
                    from markov.log_handler.logger import Logger
                    logger = Logger(__name__, logging.ERROR).get_logger()
                    logger.error("Error updating tracker: %s", ex)

    def remove(self, model_name, relative_entity_name=""):
        """
        Remove model name with relative entity name from being updated in
        model state tracker

        Args:
            model_name (str): name of the model
            relative_entity_name (str): relative entity name
        """
        with self.lock:
            cache_key = (model_name, relative_entity_name)
            if self.model_map and cache_key in self.model_map:
                self.model_map.pop(cache_key)
                # Rebuild lists from remaining cache keys like ROS1
                self.model_names = [key[0] for key in self.model_map.keys()]
                self.relative_entity_names = [key[1] for key in self.model_map.keys()]
