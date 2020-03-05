#!/usr/bin/env python
##############################################################
#                                                            #
#   Copyright 2019 Amazon.com, Inc. or its affiliates.       #
#   All Rights Reserved.                                     #
#                                                            #
##############################################################
import logging
import sys
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from markov.utils import Logger
from markov.utils import (log_and_exit,
                          SIMAPP_EVENT_ERROR_CODE_500,
                          SIMAPP_SIMULATION_SAVE_TO_MP4_EXCEPTION)
from mp4_saving.constants import (CameraTypeParams,
                                  Mp4Parameter)
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
        self.mp4_subscription = dict()

    def _subscribe_to_image_topic(self, data, camera_type):
        """ This subscribes to the topic and uses the cv_writer to write to the local disk as mp4
        Arguments:
            data (Image): Image topic where the frames as published
            camera_type (str): Enum.name of the CameraTypeParams
        """
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
            if camera_type in self.cv2_video_writers:
                self.cv2_video_writers[camera_type].write(cv_image)
        except CvBridgeError as ex:
            LOG.info("ROS image message to cv2 error: {}".format(ex))

    def subscribe_to_save_mp4(self, req):
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
                self.mp4_subscription[name] = rospy.Subscriber(topic_name, Image,
                                                               callback=self._subscribe_to_image_topic,
                                                               callback_args=name)
            return []
        except Exception as err_msg:
            log_and_exit("Exception in the handler function to subscribe to save_mp4 download: {}".format(err_msg),
                         SIMAPP_SIMULATION_SAVE_TO_MP4_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)

    def unsubscribe_to_save_mp4(self, req):
        """ Ros service handler function used to unsubscribe from the Image topic.
        This will take care of cleaning and releasing the cv2 VideoWriter
        Arguments:
            req (req): Dummy req else the ros service throws exception
        Return:
            [] - Empty list else ros service throws exception
        """
        try:
            for camera_enum in self.camera_infos:
                name = camera_enum['name']
                if name in self.mp4_subscription:
                    self.mp4_subscription[name].unregister()
                    del self.mp4_subscription[name]
                if name in self.cv2_video_writers:
                    self.cv2_video_writers[name].release()
                    del self.cv2_video_writers[name]
            return []
        except Exception as err_msg:
            log_and_exit("Exception in the handler function to unsubscribe from save_mp4 download: {}".format(err_msg),
                         SIMAPP_SIMULATION_SAVE_TO_MP4_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)

def main(racecar_names):
    """ Main function """
    try:
        for racecar_name in racecar_names.split(','):
            agent_name = 'agent' if len(racecar_name.split("_")) == 1 else "agent_{}".format(racecar_name.split("_")[1])
            camera_info = utils.get_cameratype_params(racecar_name, agent_name)
            save_to_mp4_obj = SaveToMp4(camera_infos=[camera_info[CameraTypeParams.CAMERA_PIP_PARAMS],
                                                      camera_info[CameraTypeParams.CAMERA_45DEGREE_PARAMS],
                                                      camera_info[CameraTypeParams.CAMERA_TOPVIEW_PARAMS]],
                                        fourcc=Mp4Parameter.FOURCC.value,
                                        fps=Mp4Parameter.FPS.value,
                                        frame_size=Mp4Parameter.FRAME_SIZE.value)
            rospy.Service('/{}/save_mp4/subscribe_to_save_mp4'.format(racecar_name),
                          Empty, save_to_mp4_obj.subscribe_to_save_mp4)
            rospy.Service('/{}/save_mp4/unsubscribe_from_save_mp4'.format(racecar_name),
                          Empty, save_to_mp4_obj.unsubscribe_to_save_mp4)
    except Exception as err_msg:
        log_and_exit("Exception in save_mp4 ros node: {}".format(err_msg),
                     SIMAPP_SIMULATION_SAVE_TO_MP4_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)

if __name__ == '__main__':
    rospy.init_node('save_to_mp4', anonymous=True)
    main(sys.argv[1])
    rospy.spin()
