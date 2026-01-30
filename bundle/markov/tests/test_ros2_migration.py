#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Test script for validating the ROS2 migration of AwsSilverstoneMarkovScripts
"""

import unittest
import sys
import os
import time
import threading
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

# Add the parent directory to the path so we can import the markov module
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from markov.rclpy_constants import DEFAULT_ROS2_NODE_NAME

from markov.rclpy_wrappers import ROS2NodeManager, ServiceProxyWrapper
from markov.constants import SIMAPP_VERSION_6
from markov.domain_randomizations.constants import GazeboServiceName


class TestROS2Migration(unittest.TestCase):
    """Test class for validating ROS2 migration"""

    @classmethod
    def setUpClass(cls):
        """Set up the ROS2 node for testing"""
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
        
        # Create a test node
        cls.node = Node('test_ros2_migration_node')
        
        # Create a simple service for testing
        cls.srv = cls.node.create_service(
            Empty, 
            '/test_service',
            lambda req, resp: resp
        )
        
        # Start spinning in a separate thread
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls._spin_executor, daemon=True)
        cls.spin_thread.start()
        
        # Give time for the executor to start
        time.sleep(0.1)
    
    @classmethod
    def _spin_executor(cls):
        """Spin the executor with proper exception handling"""
        try:
            while rclpy.ok():
                cls.executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            # Safely handle exceptions during shutdown
            if rclpy.ok():  # Only log if ROS is still running
                print(f"Executor exception: {e}")

    @classmethod
    def tearDownClass(cls):
        """Clean up resources"""
        try:
            # Stop the executor first
            cls.executor.shutdown()
            
            # Wait for the spin thread to finish
            if cls.spin_thread.is_alive():
                cls.spin_thread.join(timeout=1.0)
                
            # Destroy the node
            if hasattr(cls, 'node') and cls.node is not None:
                cls.node.destroy_node()
                cls.node = None
                
            # Shutdown ROS2
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during teardown: {e}")

    def test_ros2_node_manager(self):
        """Test the ROS2NodeManager singleton"""
        # Get the instance
        node_manager = ROS2NodeManager.get_instance(DEFAULT_ROS2_NODE_NAME)
        
        # Verify it's a singleton
        node_manager2 = ROS2NodeManager.get_instance(DEFAULT_ROS2_NODE_NAME)
        self.assertEqual(node_manager, node_manager2)
        
        # Verify we can get a node
        node = node_manager.get_node()
        self.assertIsNotNone(node)

    def test_service_proxy_wrapper(self):
        """Test the ServiceProxyWrapper with a simple service"""
        # Create a service proxy wrapper
        service_proxy = ServiceProxyWrapper('/test_service', Empty, timeout_sec=1.0)
        
        # Call the service
        request = Empty.Request()
        response = service_proxy(request)
        
        # Verify we got a response
        self.assertIsNotNone(response)

    def test_simapp_version(self):
        """Test that the new SIMAPP_VERSION_6 constant is defined"""
        self.assertEqual(SIMAPP_VERSION_6, 6.0)

    def test_gazebo_service_names(self):
        """Test that Gazebo service names are updated for Harmonic"""
        # Check a few key service names
        self.assertTrue(GazeboServiceName.PAUSE_PHYSICS.value.startswith('/deepracer/'))
        self.assertTrue(GazeboServiceName.GET_MODEL_PROPERTIES.value.startswith('/deepracer/'))
        self.assertTrue(GazeboServiceName.SET_VISUAL_COLORS.value.startswith('/deepracer/'))


if __name__ == '__main__':
    unittest.main()
