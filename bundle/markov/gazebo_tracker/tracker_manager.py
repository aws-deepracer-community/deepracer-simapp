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

import threading
import logging
from rosgraph_msgs.msg import Clock
import rclpy
from markov.rclpy_constants import DEFAULT_ROS2_NODE_NAME
from rclpy.node import Node
from markov.log_handler.deepracer_exceptions import GenericRolloutException
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                          SIMAPP_EVENT_ERROR_CODE_500)
import markov.gazebo_tracker.constants as consts
from markov.log_handler.logger import Logger


logger = Logger(__name__, logging.INFO).get_logger()


class TrackerManager:
    """
    TrackerManager class
    """
    _instance_ = None

    @staticmethod
    def get_instance():
        """Method for getting a reference to the Tracker Manager object"""
        if TrackerManager._instance_ is None:
            TrackerManager()
        return TrackerManager._instance_

    def __init__(self):
        if TrackerManager._instance_ is not None:
            raise GenericRolloutException("Attempting to construct multiple TrackerManager")
        self.priority_order = [consts.TrackerPriority.HIGH, consts.TrackerPriority.NORMAL, consts.TrackerPriority.LOW]
        self.tracker_map = {}
        for priority in self.priority_order:
            self.tracker_map[priority] = set()
        self.lock = threading.RLock()
        self.last_time = 0.0

        from markov.rclpy_wrappers import ROS2NodeManager
        self.node = ROS2NodeManager.get_instance(DEFAULT_ROS2_NODE_NAME).get_node()
        from rclpy.qos import QoSProfile
        self.node.create_subscription(Clock, '/clock', self._update_sim_time, QoSProfile(depth=10))

        TrackerManager._instance_ = self

    def add(self, tracker, priority=consts.TrackerPriority.NORMAL):
        """
        Add given tracker to manager

        Args:
            tracker (AbstractTracker): tracker object
            priority (TrackerPriority): prioirity
        """
        with self.lock:
            self.tracker_map[priority].add(tracker)

    def remove(self, tracker):
        """
        Remove given tracker from manager

        Args:
            tracker (AbstractTracker): tracker
        """
        with self.lock:
            for priority in self.priority_order:
                self.tracker_map[priority].discard(tracker)

    def _update_sim_time(self, sim_time):
        """
        Callback when sim time is updated

        Args:
            sim_time (Clock): simulation time
        """
        
        # Skip if rclpy is shutting down
        if not rclpy.ok():
            return
        
        # ROS2 Clock message format: sim_time.clock.sec and sim_time.clock.nanosec
        curr_time = sim_time.clock.sec + 1.e-9 * sim_time.clock.nanosec
        if self.last_time is None:
            self.last_time = curr_time
        delta_time = curr_time - self.last_time
        lock_acquired = self.lock.acquire(False)
        if lock_acquired:
            try:
                self.last_time = curr_time
                total_trackers = 0
                for priority in self.priority_order:
                    copy_trackers = self.tracker_map[priority].copy()
                    total_trackers += len(copy_trackers)
                    for tracker in copy_trackers:
                        tracker_name = tracker.__class__.__name__
                        tracker.update_tracker(delta_time, sim_time)
            except Exception as e:
                logger.info("TrackerManager: failed _update_sim_time call")
            finally:
                self.lock.release()
        else:
            logger.info("TrackerManager: missed an _update_sim_time call")

    def get_rostime(self):
        """
        Get current simulation time

        Returns:
            Clock: current simulation time
        """
        return self.node.get_clock().now()

