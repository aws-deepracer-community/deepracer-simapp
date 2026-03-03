#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import print_function

import os
import sys
import rclpy
from rclpy.node import Node
import pytest


def get_output_file():
    for arg in sys.argv:
        if arg.startswith('--gtest_output'):
            return arg.split('=xml:')[1]

    raise RuntimeError('No output file has been passed')


class TestRunnerNode(Node):
    def __init__(self):
        super().__init__('test_runner_node')
        
    def get_test_module(self):
        # In ROS 2, we'll get the test module from environment variable or parameter
        try:
            return self.get_parameter('test_module').value
        except:
            return os.environ.get('TEST_MODULE', 'mp4_saving')


if __name__ == '__main__':
    rclpy.init()
    
    try:
        output_file = get_output_file()
        
        # Create a temporary node to get parameters
        node = TestRunnerNode()
        test_module = node.get_test_module()
        
        runner_path = os.path.dirname(os.path.realpath(__file__))
        module_path = os.path.join(runner_path, test_module)

        sys.exit(
            pytest.main([module_path, '--junitxml={}'.format(output_file),
                         '--cov={}/../scripts/'.format(runner_path)])
        )
    finally:
        if rclpy.ok():
            rclpy.shutdown()
