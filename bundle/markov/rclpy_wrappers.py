# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

'''This module is intended for any wrappers that are needed for rclpy'''

import os
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import logging
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                          SIMAPP_EVENT_ERROR_CODE_500)
from markov.constants import ROBOMAKER_CANCEL_JOB_WAIT_TIME
from markov.utils import get_node_name_suffix
from markov.rclpy_constants import DEFAULT_ROS2_NODE_NAME

from std_srvs.srv import Empty


# Per-attempt wait for a response, for callers that opt in with response_timeout_sec (see
# ServiceProxyWrapper.__call__).
SERVICE_CALL_TIMEOUT_SEC = 10.0
# History depth of a service client's response reader. Every client of a service receives every
# response the server sends, to any client, and discards the ones that are not its own when it
# takes them (rmw_fastrtps; ros2/rclcpp#2397). With KEEP_LAST 10, a client whose executor takes
# its response later than ten foreign responses arrive loses its own: the model state service
# answers about 2,000 requests a second from the two /clock-rate trackers (car_node and the
# rollout worker), so a take delayed by a few milliseconds (interpreter-lock contention, worst
# during a reset) lost the rollout worker's reply and stalled the episode for the 10 s timeout.
# 1,000 covers about half a second of delay at that rate.
SERVICE_CLIENT_QOS_DEPTH = 1000
ROS_SERVICE_ERROR_MSG_FORMAT = "ROS Service {0} call failed, Re-try count: {1}/{2}: {3}"

logger = Logger(__name__, logging.INFO).get_logger()


class ROS2NodeManager:
    """Multiton class to manage ROS2 node initialization and access"""
    _instance_map = {}
    _lock = threading.Lock()

    @staticmethod
    def get_instance(node_name):
        """Get multiton instance of the ROS2 node manager"""
        with ROS2NodeManager._lock:
            if node_name not in ROS2NodeManager._instance_map:
                ROS2NodeManager._instance_map[node_name] = ROS2NodeManager(node_name)
            return ROS2NodeManager._instance_map[node_name]

    def __init__(self, node_name=DEFAULT_ROS2_NODE_NAME):
        """Initialize ROS2 node manager"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            # Get process command line from OS using PID
            suffix = get_node_name_suffix()
            
            node_name_with_suffix = f"{node_name}_{suffix}"
            self.node = Node(node_name_with_suffix)
            # At least four workers, whatever the CPU affinity: subscription callbacks on this
            # node (for example TrackerManager's /clock callback) make blocking service calls
            # through ServiceProxyWrapper, whose responses are delivered by another worker of
            # this executor. With the affinity-derived default a single-CPU process would get
            # one worker and deadlock. This holds only while fewer callbacks block on a service
            # call at once than there are workers.
            self.executor = rclpy.executors.MultiThreadedExecutor(
                num_threads=max(4, len(os.sched_getaffinity(0))))
            self.executor.add_node(self.node)
            self.executor_thread = threading.Thread(target=self._spin_executor, daemon=True)
            self.executor_thread.start()
            logger.info(f"ROS2NodeManager initialized: {node_name_with_suffix}")
        except Exception as ex:
            logger.error(f"Failed to initialize ROS2 node manager: {ex}")
            log_and_exit("ROS2 node initialization failed",
                         SIMAPP_SIMULATION_WORKER_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500,
                         name="ROS2 Name Manager")

    def _spin_executor(self):
        """Spin the executor in a separate thread"""
        while rclpy.ok():
            try:
                # Based on: https://github.com/ros2/rclpy/issues/1547
                self.executor.spin_once(timeout_sec=0.01)
                # Yield GIL to improve performance
                time.sleep(0.001)
            except Exception as ex:
                if not rclpy.ok() or "cannot schedule new futures after shutdown" in str(ex):
                    break
                logger.error(f"Error in ROS2 executor: {ex}")
                # Brief sleep to avoid tight error loop
                time.sleep(0.1)

    def get_node(self):
        """Get the ROS2 node"""
        return self.node

    def shutdown(self):
        """Shutdown the ROS2 node"""
        if self.node:
            self.node.destroy_node()
            self.node = None
        rclpy.shutdown()


class ServiceProxyWrapper:
    """This class wraps rclpy's service client so that we can wait
       if a service throws an exception. This is required to prevent
       our metrics from being flooded since an exception is thrown by service
       calls when the cancel simulation API is called.
    """
    def __init__(self, service_name, object_type=Empty, *,
                 max_retry_attempts=5, timeout_sec=1.0,
                 wait_for_service=False, response_timeout_sec=None):
        """service_name (str): Name of the service to create a client for
           object_type (object): The object type for making a service request
           persistent (bool): flag to whether keep the connection open or not (not used in ROS2)
           max_retry_attempts (int): maximum number of retry
           timeout_sec (float): timeout in seconds for service call
           wait_for_service (bool): whether to attempt wait for the service to be available before returning
           response_timeout_sec (float or None): per-attempt limit on waiting for the response. None
               (the default) waits until the response arrives or ROS shuts down, as before. Set it
               only for idempotent services whose callers must not block: on timeout the request is
               dropped on the client side and re-sent, and the server may still run the first one.
        """

        self.node = ROS2NodeManager.get_instance(DEFAULT_ROS2_NODE_NAME).get_node()
        
        # Create a separate callback group for service clients to avoid deadlocks
        service_callback_group = MutuallyExclusiveCallbackGroup()
        
        # List all available services before trying to connect
        try:
            available_services = self.node.get_service_names_and_types()
            
            # Check if our target service is in the list
            target_found = False
            for service_name_in_list, service_types in available_services:
                if service_name in service_name_in_list:
                    target_found = True
            
            if not target_found:
                # Show some services for debugging
                for service_name_in_list, service_types in available_services:
                    if 'deepracer' in service_name_in_list or 'light' in service_name_in_list:
                        logger.debug("  - %s: %s", service_name_in_list, service_types)
            else:
                logger.debug("No services found")
                
        except Exception as e:
            logger.error(f"Error listing services: {e}")
        
        # QoS for the client's request writer and response reader. The depth bounds how many
        # responses (most of them for other clients of the same service) the reader holds before
        # the executor takes them; see SERVICE_CLIENT_QOS_DEPTH.
        service_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=SERVICE_CLIENT_QOS_DEPTH,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.client = self.node.create_client(object_type, service_name, 
                                            callback_group=service_callback_group,
                                            qos_profile=service_qos)
        
        self._service_name = service_name
        self._max_retry_attempts = max_retry_attempts
        self._timeout_sec = timeout_sec
        self._response_timeout_sec = response_timeout_sec

        retry_attempts = 0
        while wait_for_service and not target_found and retry_attempts < max_retry_attempts:
            target_found = self.client.wait_for_service(timeout_sec=timeout_sec)
            retry_attempts += 1

    def __call__(self, request):
        """ Makes a client call for the stored service
            request: The request object to pass to the service
        """
        
        try_count = 0
        while True:
            try:
                # Verify service is still available before calling
                if not self.client.service_is_ready():
                    logger.warning(f"Service {self._service_name} not ready, waiting...")
                    if not self.client.wait_for_service(timeout_sec=5.0):
                        raise Exception(f"Service {self._service_name} not available")
                
                future = self.client.call_async(request)

                # The node is spun by ROS2NodeManager's executor thread, which also delivers
                # the response, so only wait for the future here. Spinning the node from the
                # calling thread as well (rclpy.spin_once) moved it to the global executor and,
                # with several calling threads, re-entered that executor: on Jazzy that failed
                # with "generator already executing" and was hidden by the retries below; since
                # ros2/rclpy#1510 (ROS 2 Lyrical) it fails with "Executor is already spinning".
                response_ready = threading.Event()
                future.add_done_callback(lambda _: response_ready.set())
                # Optional bounded wait (response_timeout_sec): a request sent before the client
                # has matched the server can be lost, and the future then never completes. The
                # gazebo state trackers call idempotent read/write-state services while holding
                # TrackerManager's lock, so for them an unbounded wait blocks the whole rollout
                # worker; they opt in and the request is dropped and re-sent by the retry loop
                # below. Other callers (spawning, video finalization) keep waiting.
                limit = self._response_timeout_sec
                deadline = None if limit is None else time.monotonic() + limit
                while not future.done() and rclpy.ok() and (deadline is None or time.monotonic() < deadline):
                    response_ready.wait(0.1)
                if not future.done() and deadline is not None:
                    self.client.remove_pending_request(future)
                    raise TimeoutError(f"no response within {limit} s")
                
                # Check why the loop exited
                if not rclpy.ok():
                    raise Exception(f"Service call to {self._service_name} aborted: rclpy shutdown")
                
                # Check future state properly
                if future.cancelled():
                    raise Exception(f"Service call to {self._service_name} was cancelled")
                
                if future.exception() is not None:
                    raise Exception(f"Service call to {self._service_name} raised exception: {future.exception()}")
                
                response = future.result()
                if response is not None:
                    return response
                else:
                    raise Exception(f"Service call to {self._service_name} returned None (service may not be ready)")
                
            except TypeError as err:
                log_and_exit(f"Invalid arguments for client {err}",
                             SIMAPP_SIMULATION_WORKER_EXCEPTION,
                             SIMAPP_EVENT_ERROR_CODE_500,
                             name=str(err))
            except Exception as ex:
                # If rclpy is shutting down, don't retry or log_and_exit - just return quietly
                if not rclpy.ok() or "shutdown" in str(ex).lower():
                    logger.info(f"Service call to {self._service_name} aborted due to shutdown")
                    return None
                
                try_count += 1
                if try_count > self._max_retry_attempts:
                    time.sleep(ROBOMAKER_CANCEL_JOB_WAIT_TIME)
                    log_and_exit(f"Unable to call service {ex}",
                                 SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                 SIMAPP_EVENT_ERROR_CODE_500,
                                 name=str(ex))
                    return None
                
                error_message = ROS_SERVICE_ERROR_MSG_FORMAT.format(self._service_name,
                                                                    str(try_count),
                                                                    str(self._max_retry_attempts),
                                                                    ex)
                logger.info(error_message)
                # Wait before retrying
                time.sleep(1.0)

    @classmethod
    def wait_for_service(cls, service_name, object_type=Empty, *, max_retry_attempts=20, timeout_sec=5.0):
        _ = ServiceProxyWrapper(service_name, object_type, max_retry_attempts=max_retry_attempts, timeout_sec=timeout_sec)

