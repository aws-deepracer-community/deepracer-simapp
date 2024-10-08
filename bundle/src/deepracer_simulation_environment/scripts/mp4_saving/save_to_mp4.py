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
import logging
import time
import sys
import threading
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_SAVE_TO_MP4_EXCEPTION)
from markov.utils import get_racecar_names
from mp4_saving import utils

LOG = Logger(__name__, logging.INFO).get_logger()

class SaveToMp4(object):
    """ SaveToMp4 class is used to subscribe/unsubscribe to the camera Image topic and
    save the frame information to the disk
    """
    def __init__(self, camera_infos, fourcc, fps, frame_size):
        self.camera_infos = camera_infos
        self.fourcc = fourcc
        self.fps = fps
        self.frame_size = frame_size
        self.cv2_video_writers = dict()
        # The SaveToMp4 would crash and bring down the entire application causing a fault.
        # In the ROS logs observed that
        #       topic impl's ref count is zero, deleting topic /racecar_0/deepracer/kvs_stream...
        #       topic[/racecar_0/deepracer/kvs_stream] removing connection to http://169.254.1.3:42335/
        # Then when unsubscribe service request is made fails with below error.
        #       Unknown error initiating TCP/IP socket to 169.254.1.3:38623
        # To fix this, we shouldn't really release the subscribe of ROS topic,
        # but just stop the mp4 saving. So reference count doesnt go to zero.
        self.mp4_subscription = dict()
        self.mp4_subscription_lock_map = dict()

    def _subscribe_to_image_topic(self, data, camera_type):
        """ This subscribes to the topic and uses the cv_writer to write to the local disk as mp4
        Arguments:
            data (Image): Image topic where the frames as published
            camera_type (str): Enum.name of the CameraTypeParams
        """
        lock_acquired = self.mp4_subscription_lock_map[camera_type].acquire(False) \
            if camera_type in self.mp4_subscription_lock_map else False

        if lock_acquired:
            try:
                if camera_type in self.cv2_video_writers:
                    bridge = CvBridge()
                    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
                    self.cv2_video_writers[camera_type].write(cv_image)
            except CvBridgeError as ex:
                LOG.info("ROS image message to cv2 error: {}".format(ex))
            except Exception as ex:
                LOG.info("Failed to save the image frame to local file: {}".format(ex))
            finally:
                self.mp4_subscription_lock_map[camera_type].release()

    def subscribe_to_save_mp4(self):
        """ Ros service handler function used to subscribe to the Image topic.
        Arguments:
            req (req): Dummy req else the ros service throws exception
        Return:
            [] - Empty list else ros service throws exception
        """
        try:
            for camera_enum in self.camera_infos:
                name = camera_enum['name']
                local_path, topic_name = camera_enum['local_path'], camera_enum['topic_name']
                self.cv2_video_writers[name] = cv2.VideoWriter(local_path, self.fourcc,
                                                               self.fps, self.frame_size)
                if name not in self.mp4_subscription or self.mp4_subscription[name] is None:
                    self.mp4_subscription[name] = rospy.Subscriber(topic_name, Image,
                                                                   callback=self._subscribe_to_image_topic,
                                                                   callback_args=name)
                if name not in self.mp4_subscription_lock_map:
                    self.mp4_subscription_lock_map[name] = threading.Lock()
                else:
                    self.mp4_subscription_lock_map[name].release()
        except Exception as err_msg:
            log_and_exit("Exception in the handler function to subscribe to save_mp4 download: {}".format(err_msg),
                         SIMAPP_SIMULATION_SAVE_TO_MP4_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)

    def unsubscribe_to_save_mp4(self, camera_enum_names):
        """ Function used to unsubscribe from the Image topic given the camera_enum_name
        This will take care of cleaning and releasing the cv2 VideoWriter
        Arguments:
            camera_enum_name (list): List of the CameraTypeParams names
        """
        try:
            for name in camera_enum_names:
                if name in self.mp4_subscription_lock_map:
                    self.mp4_subscription_lock_map[name].acquire()
                if name in self.cv2_video_writers:
                    self.cv2_video_writers[name].release()
                    del self.cv2_video_writers[name]
        except Exception as err_msg:
            log_and_exit("Exception in the handler function to unsubscribe from save_mp4 download: {}".format(err_msg),
                         SIMAPP_SIMULATION_SAVE_TO_MP4_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)
