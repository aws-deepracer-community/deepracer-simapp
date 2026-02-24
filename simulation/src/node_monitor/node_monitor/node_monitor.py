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
"""A class for NodeMonitor & interface NodeMonitorObserverInterface"""
import fnmatch
import logging
import time
import os

import rosnode

from typing import Optional, Set, List, TypeVar
from threading import RLock
import json
from markov.log_handler.constants import (EXCEPTION_HANDLER_SYNC_FILE,
                                          STOP_ROS_NODE_MONITOR_SYNC_FILE,
                                          SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_ROS_NODE_MONITOR_EXCEPTION,
                                          CLOUDWATCH_LOG_WORKER_SLEEP_TIME)

from markov.log_handler.exception_handler import log_and_exit, kill_sagemaker_simapp_jobs_by_pid
from markov.utils import cancel_simulation_job
from markov.constants import DEEPRACER_JOB_TYPE_ENV, DeepRacerJobType

NodeMonitorObserverInterface = TypeVar('NodeMonitorObserverInterface')

class NodeMonitor(object):
    """
    NodeMonitor Class.
    """

    def __init__(self, monitor_nodes: Optional[List[str]] = None,
                 update_rate_hz: float = 2) -> None:
        """
        Initialize NodeMonitor.

        Args:
            monitor_nodes (Optional[List[str]]): List of nodes to be monitored.
                If None, then all running nodes will be monitored (default: None)
            update_rate_hz (float): Rate at which nodes status is monitored (default: 0.2)
        """
        self._monitor_nodes = monitor_nodes
        self._update_rate_hz = update_rate_hz
        self._running_nodes = set()
        self._dead_nodes = set()

        self._observer_lock = RLock()
        self._running_node_lock = RLock()
        self._dead_node_lock = RLock()

        self._observers = set()
        self._seen_nodes = set()
        self._is_monitoring = False

    def register(self, observer: NodeMonitorObserverInterface) -> None:
        """
        Observers of the NodeMonitor call this class to register. If any update
        in the NodeMonitor class, then it will be notified to all the observer class.

        Args:
            observer (NodeMonitorObserverInterface): Instance of the NodeMonitorObserverInterface class,
                which wants to listen to any update from the NodeMonitor class.
        """
        with self._observer_lock:
            self._observers.add(observer)
        logging.debug("[NodeMonitor]: Registered an observer")

    def unregister(self, observer: NodeMonitorObserverInterface) -> None:
        """
        Observers of the NodeMonitor class call this class to unregister. If any update
        in the NodeMonitor class will not be notified to the observer class going forward.

        Args:
            observer (NodeMonitorObserverInterface): Instance of the NodeMonitorObserverInterface class,
                which does not want to listen to any update from the NodeMonitor class going forward.
        """
        with self._observer_lock:
            self._observers.discard(observer)
        logging.debug("[NodeMonitor]: Unregistered an observer")

    def _is_monitor_node(self, node_name: str) -> bool:
        """
        Returns boolean value if node matches the monitor_node pattern

        Args:
            node_name (str): Node name that has to be checked with the monitor node list

        Returns
            bool: Returns true if node_name matches the pattern in monitor node list
        """
        for monitor_node in self._monitor_nodes:
            if fnmatch.fnmatch(node_name, monitor_node):
                return True
        return False

    def _update_running_nodes(self) -> None:
        """
        Helper function to find all the running nodes and update the observers if change in running nodes.

        Although the rosnode.get_node_names gives the list of node names, it is no gurantee that the nodes
        are actually running. Pinging the nodes is the best approach.
        http://docs.ros.org/en/jade/api/rosnode/html/rosnode-pysrc.html#rosnode_ping_all
        Pings all the running nodes

        Add to seen_nodes and running_nodes if node is present in monitor_nodes or monitor_node is empty
        """
        with self._running_node_lock:
            try:
                (ping_nodes, _) = rosnode.rosnode_ping_all()      
            except Exception as ex:
                ping_nodes = []
                logging.warn("[NodeMonitor] Unable to use rosnode ping: {}".format(str(ex)))  
            # Since the monitor node is empty, defaulting to monitoring all nodes
            if not self._monitor_nodes:
                self._seen_nodes = set(ping_nodes)
                self._running_nodes = set(ping_nodes)
            else:
                current_running_nodes = set()
                for node in ping_nodes:
                    if self._is_monitor_node(node):
                        # Update seen nodes if node regex matches monitor nodes
                        if node not in self._seen_nodes:
                            self._seen_nodes.add(node)
                        # Add to current running nodes
                        current_running_nodes.add(node)
                self._running_nodes = current_running_nodes

    def _update_dead_nodes(self) -> None:
        """
        Helper function to find all the dead nodes and update the observers if dead node values are updated.

        Dead nodes can be found by `dead_nodes = seen_nodes - running_nodes`
        """
        with self._dead_node_lock:
            self._dead_nodes = self._seen_nodes - self._running_nodes

    def start(self) -> None:
        """
        Starts node monitoring.

        Args:
            observer (NodeMonitorObserverInterface): Instance of the NodeMonitorObserverInterface class,
                which wants to start monitoring ROS nodes
        """
        for observers in self._observers:
            observers.on_start(self)
        self._is_monitoring = True
        try:
            while self._is_monitoring:
                if(self.checkIfROSNodeMonitorShouldStop()):
                    for observers in self._observers:
                        observers.on_job_successful_completion(self)
                    self.stopSimulationJob()
                    break
                is_status_changed = False
                prev_running_nodes = self._running_nodes.copy()
                prev_dead_nodes = self._dead_nodes.copy()
                self._update_running_nodes()
                self._update_dead_nodes()
                if prev_dead_nodes != self._dead_nodes:
                    is_status_changed = True
                    for observers in self._observers:
                        observers.on_dead_node_update(self, self._dead_nodes)
                    self.stopSimulationJob()
                if prev_running_nodes != self._running_nodes:
                    is_status_changed = True
                    for observers in self._observers:
                        observers.on_running_node_update(self, self._running_nodes)
                # unit testing infinite loops can be a pain. Having a time.sleep makes
                # it easier to write tests
                time.sleep(1.0 / self._update_rate_hz)
        except Exception as ex:
            logging.error("[NodeMonitor]: NodeMonitor start method threw exception: {}".format(str(ex)))
            # Stop the simapp job when ros node monitor throws an exception
            log_and_exit("[NodeMonitor]: NodeMonitor start method threw exception: ",
                     SIMAPP_ROS_NODE_MONITOR_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500, stopRosNodeMonitor=False)
            self.stopSimulationJob()
        finally:
            logging.info("[NodeMonitor] Node monitoring closed")

    def stopSimulationJob(self) -> None:
        # The sleep time is for logs to get uploaded to cloudwatch
        time.sleep(CLOUDWATCH_LOG_WORKER_SLEEP_TIME)
        logging.info("[NodeMonitor] Cancelling sim job")
        if os.environ.get(DEEPRACER_JOB_TYPE_ENV) == DeepRacerJobType.SAGEONLY.value:
            logging.info("simapp_exit_gracefully - Job type is SageOnly. Killing SimApp and Training jobs by PID")
            kill_sagemaker_simapp_jobs_by_pid()
        cancel_simulation_job()
        self.stop()
    
    def checkIfROSNodeMonitorShouldStop(self) -> bool:
        if (os.path.isfile(STOP_ROS_NODE_MONITOR_SYNC_FILE)):
            try:
                with open(STOP_ROS_NODE_MONITOR_SYNC_FILE, 'r') as f:
                    STOP_NODE_MONITOR = json.loads(f.read())
                if(STOP_NODE_MONITOR["stop_ros_node_monitor"] == "True"):
                    logging.info("[NodeMonitor] Stopping ROS node monitor")
                    return True
                logging.info("[NodeMonitor] Uncertain value for STOP_NODE_MONITOR: {}".format(STOP_NODE_MONITOR))
            except Exception as ex:
                logging.error("[NodeMonitor] Exception when reading file for STOP_NODE_MONITOR: {}".format(str(ex)))
        return False
    
    def stop(self) -> None:
        """
        Stop node monitoring.

        Args:
            observer (NodeMonitorObserverInterface): Instance of the NodeMonitorObserverInterface class,
                which wants to stop node monitoring ROS nodes
        """
        logging.info("[NodeMonitor] Stopping Node monitoring")
        for observers in self._observers:
            observers.on_stop(self)
        self._is_monitoring = False

    @property
    def monitor_nodes(self) -> List[str]:
        """
        Returns the list of nodes to be monitored

        Returns:
            List[str]: list of nodes to be monitored
        """
        return self._monitor_nodes.copy()

    @property
    def update_rate_hz(self) -> float:
        """
        Returns the rate at which nodes status is monitored.

        Returns:
            float: rate at which node status is monitored
        """
        return self._update_rate_hz

    @property
    def running_nodes(self) -> Set[str]:
        """
        Returns the list of running ROS nodes

        Returns:
            Set[str]: list of ROS nodes running.
        """
        return self._running_nodes.copy()

    @property
    def dead_nodes(self) -> Set[str]:
        """
        Returns the list of dead ROS nodes

        Returns:
            Set[str]: list of ROS nodes that are dead.
        """
        return self._dead_nodes.copy()
