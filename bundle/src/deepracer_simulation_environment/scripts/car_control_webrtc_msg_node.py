#!/usr/bin/env python3
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

""" Ros module for processing the incoming car control message. """
import sys
import logging
import json
from collections import OrderedDict
import rospy
from std_msgs.msg import String
from markov.log_handler.logger import Logger
from markov.log_handler.deepracer_exceptions import GenericNonFatalException
from markov.virtual_event.constants import (WEBRTC_DATA_PUB_TOPIC,
                                            WEBRTC_CAR_CTRL_FORMAT)
from markov.virtual_event.utils import process_car_control_msg

LOG = Logger(__name__, logging.INFO).get_logger()


class VirtualEventCarControl(object):
    """
    VirtualEventCarControl class
    """
    def __init__(self, racecar_name):
        """
        VirtualEventCarControl constructor

        Args:
            racecar_name (str): racecar name
        """
        self._racecar_name = racecar_name
        self._control_publisher_dict = OrderedDict()
        rospy.Subscriber(WEBRTC_DATA_PUB_TOPIC.format(self._racecar_name),
                         String,
                         self._subscriber_callback)

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
                self._control_publisher_dict[topic] = rospy.Publisher(WEBRTC_CAR_CTRL_FORMAT.format(self._racecar_name, topic),
                                                                      String, queue_size=1)
            self._control_publisher_dict[topic].publish(json.dumps(payload))
            LOG.info('[webrtc msg process] published to topic %s', WEBRTC_CAR_CTRL_FORMAT.format(self._racecar_name, topic))
        except GenericNonFatalException as ex:
            ex.log_except_and_continue()


if __name__ == '__main__':
    rospy.init_node('car_control_webrtc_node', anonymous=True)
    VirtualEventCarControl(racecar_name=sys.argv[1])
    rospy.spin()
