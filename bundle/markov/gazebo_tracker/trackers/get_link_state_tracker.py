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

import logging
import threading
import time
from markov.log_handler.deepracer_exceptions import GenericRolloutException
from markov.log_handler.logger import Logger
import copy

from deepracer_msgs.srv import GetLinkStates
from markov.track_geom.constants import GET_LINK_STATES
from markov.rclpy_wrappers import ServiceProxyWrapper
from markov.gazebo_tracker.abs_tracker import AbstractTracker
import markov.gazebo_tracker.constants as consts

logger = Logger(__name__, logging.INFO).get_logger()


class GetLinkState_Response:
    def __init__(self):
        self.success = False
        self.status_message = ''
        self.link_state = None


class GetLinkStateTracker(AbstractTracker):
    """
    GetLinkState Tracker class
    """
    _instance_ = None

    @staticmethod
    def get_instance():
        """Method for getting a reference to the GetLinkState Tracker object"""
        if GetLinkStateTracker._instance_ is None:
            GetLinkStateTracker()
        return GetLinkStateTracker._instance_

    def __init__(self):
        if GetLinkStateTracker._instance_ is not None:
            raise GenericRolloutException("Attempting to construct multiple GetLinkState Tracker")

        self.lock = threading.RLock()
        self.link_map = {}
        self.link_names = []
        self.reference_frames = []

        self._get_link_states = ServiceProxyWrapper(GET_LINK_STATES, GetLinkStates)

        GetLinkStateTracker._instance_ = self
        super(GetLinkStateTracker, self).__init__(priority=consts.TrackerPriority.HIGH)

    def get_link_state(self, link_name, reference_frame, blocking=False,
                       auto_sync=True):
        """
        Return link state of given link name based on given reference frame

        Args:
            link_name (str): name of the link
            reference_frame (str): reference frame
            blocking (bool): flag to block or not
            auto_sync (bool): flag whether to automatically synchronize or not.
                              - Ignored if (model_name, relative_entity_name) pair is already using auto_sync

        Returns:
            response msg (GetLinkState_Response)
        """
        call_time = time.time()
        
        msg = GetLinkState_Response()
        msg.success = True
        key = (link_name, reference_frame)
        with self.lock:
            if blocking or key not in self.link_map:
                # Create ROS2 service request object
                request = GetLinkStates.Request()
                request.link_names = [key[0]]
                request.reference_frames = [key[1]]
                res = self._get_link_states(request)
                
                # Debug the service response
                if hasattr(res, 'link_states') and len(res.link_states) > 0:
                    link_state = res.link_states[0]
                    logger.debug("Link state position: x=%.3f, y=%.3f, z=%.3f",
                               link_state.pose.position.x if link_state else 0.0,
                               link_state.pose.position.y if link_state else 0.0, 
                               link_state.pose.position.z if link_state else 0.0)
                    
                    # Check if position is at origin (indicates car not properly spawned)
                    if link_state and link_state.pose.position.x == 0.0 and link_state.pose.position.y == 0.0:
                        logger.warning("Link state at origin - car may not be properly spawned")
                        
                if res.success and res.status[0]:
                    msg.link_state = res.link_states[0]
                    if auto_sync or key in self.link_map:
                        if key not in self.link_map:
                            self.link_names.append(link_name)
                            self.reference_frames.append(reference_frame)
                        self.link_map[key] = copy.deepcopy(msg.link_state)
                else:
                    msg.success = False
                    msg.status_message = res.messages[0] if res.success else res.status_message
            else:
                msg.link_state = copy.deepcopy(self.link_map[key])
        return msg

    def update_tracker(self, delta_time, sim_time):
        """
        Update link_states of the links that this tracker is tracking

        Args:
            delta_time (float): delta time
            sim_time (Clock): simulation time
        """
        if self.link_names:
            with self.lock:
                # Create ROS2 service request object
                request = GetLinkStates.Request()
                request.link_names = self.link_names
                request.reference_frames = self.reference_frames
                res = self._get_link_states(request)
                if res.success:
                    for link_state, status in zip(res.link_states, res.status):
                        if status:
                            link_name = link_state.link_name
                            reference_frame = link_state.reference_frame
                            key = (link_name, reference_frame)
                            self.link_map[key] = link_state
