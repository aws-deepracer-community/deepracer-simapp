#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

""" Ros module for processing the incoming car control message. """
import sys
import logging
import json
from collections import OrderedDict
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from markov.log_handler.logger import Logger
from markov.log_handler.deepracer_exceptions import GenericNonFatalException
from markov.virtual_event.constants import (WEBRTC_DATA_PUB_TOPIC,
                                            WEBRTC_CAR_CTRL_FORMAT)
from markov.virtual_event.utils import process_car_control_msg

LOG = Logger(__name__, logging.INFO).get_logger()


class VirtualEventCarControlNode(Node):
    """
    VirtualEventCarControl class
    """
    def __init__(self, racecar_name):
        """
        VirtualEventCarControl constructor

        Args:
            racecar_name (str): racecar name
        """
        super().__init__('car_control_webrtc_node')
        self._racecar_name = racecar_name
        self._control_publisher_dict = OrderedDict()
        
        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            WEBRTC_DATA_PUB_TOPIC.format(self._racecar_name),
            self._subscriber_callback,
            QoSProfile(depth=10)
        )

    def _subscriber_callback(self, message):
        """
        subscriber callback to parse car control message and publish

        Args:
            message (std_msgs.msg.String): The WebRTC incoming message in String format.
        """
        try:
            topic, payload = process_car_control_msg(message.data)
            LOG.info("[webrtc msg process] Processed message, topic: %s payload: %s", topic, payload)
            if topic not in self._control_publisher_dict:
                self._control_publisher_dict[topic] = self.create_publisher(
                    String, 
                    WEBRTC_CAR_CTRL_FORMAT.format(self._racecar_name, topic),
                    10
                )
            
            # Create message and publish
            msg = String()
            msg.data = json.dumps(payload)
            self._control_publisher_dict[topic].publish(msg)
            LOG.info('[webrtc msg process] published to topic %s', WEBRTC_CAR_CTRL_FORMAT.format(self._racecar_name, topic))
        except GenericNonFatalException as ex:
            ex.log_except_and_continue()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        racecar_name = sys.argv[1] if len(sys.argv) > 1 else 'racecar'
        node = VirtualEventCarControlNode(racecar_name=racecar_name)
        rclpy.spin(node)
    except Exception as ex:
        LOG.error(f"Exception in car control webrtc node: {ex}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
