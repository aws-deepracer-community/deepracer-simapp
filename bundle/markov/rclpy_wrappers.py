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
            self.executor = rclpy.executors.MultiThreadedExecutor()
            self.executor.add_node(self.node)
            self.executor_thread = threading.Thread(target=self._spin_executor, daemon=True)
            self.executor_thread.start()
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
                self.executor.spin_once(timeout_sec=0.001)
                # Yield GIL to improve performance
                time.sleep(0)
            except Exception as ex:
                logger.error(f"Error in ROS2 executor: {ex}")
                # Brief sleep to avoid tight error loop
                time.sleep(0.01)

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
                 wait_for_service=False):
        """service_name (str): Name of the service to create a client for
           object_type (object): The object type for making a service request
           persistent (bool): flag to whether keep the connection open or not (not used in ROS2)
           max_retry_attempts (int): maximum number of retry
           timeout_sec (float): timeout in seconds for service call
           wait_for_service (bool): whether to attempt wait for the service to be available before returning
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
        
        # Create optimized QoS profile for service calls
        # Based on: https://github.com/ros2/rclpy/issues/630
        service_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,  # Prevent service call queue overflow
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.client = self.node.create_client(object_type, service_name, 
                                            callback_group=service_callback_group,
                                            qos_profile=service_qos)
        
        self._service_name = service_name
        self._max_retry_attempts = max_retry_attempts
        self._timeout_sec = timeout_sec

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
                # start_time = time.time() # Renable to profile service call duration
                
                future = self.client.call_async(request)
                
                # Use optimized spin_once with timeout instead of spin_until_future_complete
                # This prevents GIL blocking and performance degradation
                # Based on: https://github.com/ros2/rclpy/issues/1223
                while not future.done() and rclpy.ok():
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                    time.sleep(0.001)  # Yield GIL to prevent thread starvation
                
                # elapsed_time = time.time() - start_time # Renable to profile service call duration
                # logger.info(f"PROFILE: Service {self._service_name} took {elapsed_time*1000:.1f}ms") # Renable to profile service call duration
                
                
                
                if future.result() is not None:
                    response = future.result()
                    return response
                else:
                    raise Exception(f"Service call to {self._service_name} returned None")
                
            except TypeError as err:
                log_and_exit(f"Invalid arguments for client {err}",
                             SIMAPP_SIMULATION_WORKER_EXCEPTION,
                             SIMAPP_EVENT_ERROR_CODE_500,
                             name=str(err))
            except Exception as ex:
                try_count += 1
                if try_count > self._max_retry_attempts:
                    time.sleep(ROBOMAKER_CANCEL_JOB_WAIT_TIME)
                    log_and_exit(f"Unable to call service {ex}",
                                 SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                 SIMAPP_EVENT_ERROR_CODE_500,
                                 name=str(ex))
                
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

