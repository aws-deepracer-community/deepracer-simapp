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

import rclpy
from rclpy.node import Node
from rclpy.parameter_service import ParameterService
from rcl_interfaces.srv import ListParameters

from typing import Optional, Set, List, TypeVar, Tuple
from threading import RLock
import json
from markov.log_handler.constants import (
    EXCEPTION_HANDLER_SYNC_FILE,
    STOP_ROS_NODE_MONITOR_SYNC_FILE,
    SIMAPP_EVENT_ERROR_CODE_500,
    SIMAPP_ROS_NODE_MONITOR_EXCEPTION,
    CLOUDWATCH_LOG_WORKER_SLEEP_TIME,
)

from markov.log_handler.exception_handler import (
    log_and_exit,
    kill_sagemaker_simapp_jobs_by_pid,
)
from markov.utils import cancel_simulation_job
from markov.constants import DEEPRACER_JOB_TYPE_ENV, DeepRacerJobType

NodeMonitorObserverInterface = TypeVar("NodeMonitorObserverInterface")


class NodeMonitor(object):
    """
    NodeMonitor Class.
    """

    def __init__(
        self, monitor_nodes: Optional[List[str]] = None, update_rate_hz: float = 2
    ) -> None:
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
        self._service_clients_lock = RLock()

        self._observers = set()
        self._seen_nodes = set()
        self._is_monitoring = False
        self._service_clients = {}
        
        # Initialize ROS2 node for client operations
        if not rclpy.ok():
            rclpy.init()
        self._ros_node = Node('node_monitor')

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

    def _get_service_client(self, service_name: str):
        """Return cached service client for a service name, creating it if needed."""
        with self._service_clients_lock:
            client = self._service_clients.get(service_name)
            if client is None:
                client = self._ros_node.create_client(ListParameters, service_name)
                self._service_clients[service_name] = client
            return client

    def _destroy_service_client(self, service_name: str) -> None:
        """Destroy and remove a cached service client if present."""
        with self._service_clients_lock:
            client = self._service_clients.pop(service_name, None)
            if client is not None:
                try:
                    self._ros_node.destroy_client(client)
                except Exception as ex:
                    logging.exception(f"[NodeMonitor] Failed to destroy service client {service_name}: {ex}")

    def _prune_stale_service_clients(self, active_service_names: Set[str]) -> None:
        """Destroy cached clients that were not observed in the latest node graph snapshot."""
        with self._service_clients_lock:
            stale_service_names = [
                service_name
                for service_name in self._service_clients
                if service_name not in active_service_names
            ]
            for service_name in stale_service_names:
                client = self._service_clients.pop(service_name, None)
                if client is not None:
                    try:
                        self._ros_node.destroy_client(client)
                    except Exception as ex:
                        logging.exception(
                            f"[NodeMonitor] Failed to destroy stale service client {service_name}: {ex}"
                        )

    def _destroy_all_service_clients(self) -> None:
        """Destroy all cached service clients."""
        with self._service_clients_lock:
            for service_name, client in self._service_clients.items():
                try:
                    self._ros_node.destroy_client(client)
                except Exception as ex:
                    logging.exception(
                        f"[NodeMonitor] Failed to destroy service client {service_name}: {ex}"
                    )
            self._service_clients.clear()

    def _ros2_ping_all(self) -> List[str]:
        """
        Custom function to list active ROS2 nodes and verify they're responsive.

        Returns:
            List[str]: List of active node names that respond to service calls
        """
        if not self._is_monitoring:
            return []
        
        active_nodes = []
        try:
            # Get all node names from ROS2
            node_names_and_namespaces = self._ros_node.get_node_names_and_namespaces()
            active_service_names = set()

            # For each node, try to call the list_parameters service
            for node_name, namespace in node_names_and_namespaces:
                # Create the fully qualified node name
                if namespace == "/":
                    full_node_name = f"/{node_name}"
                else:
                    full_node_name = f"{namespace}/{node_name}"

                # Reuse a client for the list_parameters service
                service_name = f"{full_node_name}/list_parameters"
                active_service_names.add(service_name)
                try:
                    client = self._get_service_client(service_name)
                    # Wait for service to be available (with timeout)
                    if client.wait_for_service(timeout_sec=5.0):
                        # Create and send request
                        request = ListParameters.Request()
                        future = client.call_async(request)

                        # Process the service call (with timeout)
                        start_time = time.time()
                        while time.time() - start_time < 5.0:
                            rclpy.spin_once(self._ros_node, timeout_sec=0.1)
                            if future.done():
                                try:
                                    # If we get a response, the node is active
                                    future.result()
                                    active_nodes.append(full_node_name)
                                except Exception as ex:
                                    # If the service call fails, the node is not considered active (expected condition)
                                    logging.debug(f"[NodeMonitor] Service call failed for {full_node_name}: {ex}")
                                break
                except Exception as ex:
                    # Unexpected error during service query - capture full traceback
                    logging.exception(
                        f"[NodeMonitor] Error while querying service {service_name}: {str(ex)}"
                    )
                    self._destroy_service_client(service_name)

            try:
                self._prune_stale_service_clients(active_service_names)
            except Exception as ex:
                logging.exception(
                    f"[NodeMonitor] Failed to prune stale service clients: {ex}"
                )

            return active_nodes
        except Exception as ex:
            logging.warning(f"[NodeMonitor] Error getting ROS2 nodes: {str(ex)}")
            return []

    def _update_running_nodes(self) -> None:
        """
        Helper function to find all the running nodes and update the observers if change in running nodes.

        Uses ROS2 node discovery and service calls to determine which nodes are active.
        Add to seen_nodes and running_nodes if node is present in monitor_nodes or monitor_node is empty
        """
        with self._running_node_lock:
            try:
                ping_nodes = self._ros2_ping_all()
            except Exception as ex:
                ping_nodes = []
                logging.warning(f"[NodeMonitor] Unable to ping ROS2 nodes: {str(ex)}")

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
                if self.checkIfROSNodeMonitorShouldStop():
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
                    newly_dead_nodes = self._dead_nodes - prev_dead_nodes
                    logging.error("[NodeMonitor] DEAD NODES DETECTED! Newly dead: {}, All dead: {}, All running: {}".format(
                        list(newly_dead_nodes), list(self._dead_nodes), list(self._running_nodes)))
                    for observers in self._observers:
                        observers.on_dead_node_update(self, self._dead_nodes)
                    self.stopSimulationJob()
                if prev_running_nodes != self._running_nodes:
                    is_status_changed = True
                    newly_running_nodes = self._running_nodes - prev_running_nodes
                    stopped_running_nodes = prev_running_nodes - self._running_nodes
                    logging.debug("[NodeMonitor] NODE STATUS CHANGE - New: {}, Stopped: {}, Total running: {}".format(
                        list(newly_running_nodes), list(stopped_running_nodes), list(self._running_nodes)))
                    for observers in self._observers:
                        observers.on_running_node_update(self, self._running_nodes)
                # unit testing infinite loops can be a pain. Having a time.sleep makes
                # it easier to write tests
                time.sleep(1.0 / self._update_rate_hz)
        except Exception as ex:
            logging.error(
                "[NodeMonitor]: NodeMonitor start method threw exception: {}".format(
                    str(ex)
                )
            )
            # Stop the simapp job when ros node monitor throws an exception
            log_and_exit(
                "[NodeMonitor]: NodeMonitor start method threw exception: ",
                SIMAPP_ROS_NODE_MONITOR_EXCEPTION,
                SIMAPP_EVENT_ERROR_CODE_500,
                stopRosNodeMonitor=False,
            )
            self.stopSimulationJob()
        finally:
            logging.info("[NodeMonitor] Node monitoring closed")

    def stopSimulationJob(self) -> None:
        # The sleep time is for logs to get uploaded to cloudwatch
        time.sleep(CLOUDWATCH_LOG_WORKER_SLEEP_TIME)
        logging.info("[NodeMonitor] Cancelling sim job")
        if os.environ.get(DEEPRACER_JOB_TYPE_ENV) == DeepRacerJobType.SAGEONLY.value:
            logging.info(
                "simapp_exit_gracefully - Job type is SageOnly. Killing SimApp and Training jobs by PID"
            )
            kill_sagemaker_simapp_jobs_by_pid()
        cancel_simulation_job()
        self.stop()

    def checkIfROSNodeMonitorShouldStop(self) -> bool:
        if os.path.isfile(STOP_ROS_NODE_MONITOR_SYNC_FILE):
            try:
                with open(STOP_ROS_NODE_MONITOR_SYNC_FILE, "r") as f:
                    STOP_NODE_MONITOR = json.loads(f.read())
                if STOP_NODE_MONITOR["stop_ros_node_monitor"] == "True":
                    logging.info("[NodeMonitor] Stopping ROS node monitor")
                    return True
                logging.info(
                    "[NodeMonitor] Uncertain value for STOP_NODE_MONITOR: {}".format(
                        STOP_NODE_MONITOR
                    )
                )
            except Exception as ex:
                logging.error(
                    "[NodeMonitor] Exception when reading file for STOP_NODE_MONITOR: {}".format(
                        str(ex)
                    )
                )
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

        self._destroy_all_service_clients()
        
        # Clean up ROS node
        if hasattr(self, '_ros_node') and self._ros_node is not None:
            self._ros_node.destroy_node()

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
