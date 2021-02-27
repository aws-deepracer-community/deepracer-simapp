#!/usr/bin/env python
""" Ros module for processing the incoming car control message. """
import logging
import json
from collections import OrderedDict
from datetime import datetime
import rospy
from std_msgs.msg import String
from markov.log_handler.logger import Logger
from markov.log_handler.deepracer_exceptions import GenericNonFatalException
from markov.virtual_event.constants import (WEBRTC_DATA_PUB_TOPIC,
                                            WEBRTC_CAR_CTRL_FORMAT)
from markov.virtual_event.utils import process_car_control_msg
from markov.log_handler.constants import (SIMAPP_EVENT_SYSTEM_ERROR,
                                          SIMAPP_EVENT_USER_ERROR,
                                          SIMAPP_EVENT_ERROR_CODE_400,
                                          SIMAPP_EVENT_ERROR_CODE_500)

LOG = Logger(__name__, logging.INFO).get_logger()


class CarControlWebRTCMessageProcessor(object):
    """Ros node for processing incoming webrtc message
       and publishing car control to the appropriate topic.
    """
    def __init__(self, webrtc_topic):
        """Initialize a CarControlWebRTCMessageProcessor.

        Args:
            webrtc_topic (str): the webrtc topic to subscribe to.
        """
        self.webrtc_topic = webrtc_topic
        self._control_publisher_dict = OrderedDict()
        rospy.Subscriber(webrtc_topic, String, self._msg_process)

    def _msg_process(self, message):
        """ processes the WebRTC ros incoming message from the webrtc_topic.

        Args:
            message (std_msgs.msg.String): The WebRTC incoming message in String format.
        """
        try:
            topic, payload = process_car_control_msg(message.data)
            LOG.info("[webrtc msg process] Processed message, topic: %s payload: %s", topic, payload)
            if topic not in self._control_publisher_dict:
                self._control_publisher_dict[topic] = rospy.Publisher(WEBRTC_CAR_CTRL_FORMAT.format(topic),
                                                                      String, queue_size=1)
            self._control_publisher_dict[topic].publish(json.dumps(payload))
            LOG.info('[webrtc msg process] published to topic %s', WEBRTC_CAR_CTRL_FORMAT.format(topic))
        except GenericNonFatalException as ex:
            ex.log_except_and_continue()


if __name__ == '__main__':
    rospy.init_node('car_control_webrtc_node', anonymous=True)
    CarControlWebRTCMessageProcessor(WEBRTC_DATA_PUB_TOPIC)
    rospy.spin()
