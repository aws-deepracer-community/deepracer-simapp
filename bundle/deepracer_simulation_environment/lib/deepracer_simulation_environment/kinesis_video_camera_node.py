#!/usr/bin/env python
##############################################################
#                                                            #
#   Copyright 2019 Amazon.com, Inc. or its affiliates.       #
#   All Rights Reserved.                                     #
#                                                            #
##############################################################
import sys
import time
import logging
from threading import Thread
import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ROSImg
from markov.utils import DoubleBuffer, force_list, get_video_display_name, get_racecar_names
from markov.constants import DEFAULT_COLOR                     
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_KINESIS_VIDEO_CAMERA_EXCEPTION)
from markov.reset.constants import RaceType
from mp4_saving.constants import RaceCarColorToRGB
from mp4_saving.single_agent_image_editing import SingleAgentImageEditing
from mp4_saving.multi_agent_image_editing import MultiAgentImageEditing
from mp4_saving.training_image_editing import TrainingImageEditing

LOG = Logger(__name__, logging.INFO).get_logger()

CAMERA_FPS = 1.0 / 15.0

class KinesisVideoCamera(object):
    """ This node is used to produce frames for the AWS kinesis video stream and
    for saving the mp4 and uploading to S3. Both are subscribed to the output of
    the image topic produced by this node.
    """
    def __init__(self, racecar_name, racecars_info):
        self.racecar_name = racecar_name
        self.racecars_info = racecars_info
        # init cv bridge
        self.bridge = CvBridge()
        # Double buffer so that we can always publish to KVS, even if physics freezes
        self.main_camera_frame_buffer = DoubleBuffer(clear_data_on_get=False)

        main_camera_topic = "/{}/{}/zed/rgb/image_rect_color".format(racecar_name, "main_camera")

        # This the topic that the camera object publishes too
        rospy.Subscriber(main_camera_topic, ROSImg, self._main_camera_cb_)
        # Create a publisher and new topic for kvs to subscribe to
        self.kvs_pub = rospy.Publisher('/{}/deepracer/kvs_stream'.format(racecar_name), ROSImg, queue_size=1)

        # This determines what kinding of image editing should be done based on the race type
        self.job_type_image_edit = self._get_image_editing_job_type()
        # Run the publisher on its own thread
        Thread(target=self._publish_kvs_frames_).start()

    def _get_image_editing_job_type(self):
        """ This determines what kinding of image editing should be done based on the race type
        Returns:
            ImageEditingObj: Instantiating an object based on training/evaluation and racetype
        """
        race_type = rospy.get_param("RACE_TYPE", RaceType.TIME_TRIAL.value)
        is_training = rospy.get_param("JOB_TYPE") == 'TRAINING'
        if is_training:
            return TrainingImageEditing(self.racecars_info[0])
        if race_type == RaceType.HEAD_TO_MODEL.value or race_type == RaceType.F1.value:
            return MultiAgentImageEditing(self.racecar_name, self.racecars_info,
                                          race_type)
        if race_type in [RaceType.TIME_TRIAL.value, RaceType.OBJECT_AVOIDANCE.value,
                         RaceType.HEAD_TO_BOT.value]:
            return SingleAgentImageEditing(self.racecars_info[0], race_type)
        raise Exception("Unknown job type for image editing")

    def _main_camera_cb_(self, frame):
        '''Callback for the frames being publish by the top camera topic
            frame - Frames, of type Image, being published by main camera topic
        '''
        self.main_camera_frame_buffer.put(frame)

    def _overlay_two_images_(self, major_frame):
        # convert ros image message to cv image
        try:
            major_cv_image = self.bridge.imgmsg_to_cv2(major_frame, "bgr8")
        except CvBridgeError as ex:
            LOG.info("ROS image message to cv2 error: {}".format(ex))

        major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2RGBA)
        # Edit the image based on the racecar type and job type
        major_cv_image = self.job_type_image_edit.edit_image(major_cv_image)

        # convert cv image back to ros image message
        try:
            overlay_frame = self.bridge.cv2_to_imgmsg(major_cv_image, "bgr8")
        except CvBridgeError as ex:
            LOG.info("cv2 to ROS image message error: {}".format(ex))

        return overlay_frame

    def _publish_kvs_frames_(self):
        '''This method should be run in its own thread, its used to publish frames
           to the kvs encoder
        '''
        while not rospy.is_shutdown():
            main_camera_frame = self.main_camera_frame_buffer.get()
            frame = self._overlay_two_images_(main_camera_frame)
            time.sleep(CAMERA_FPS)
            if not rospy.is_shutdown():
                self.kvs_pub.publish(frame)

def get_racecars_info(racecar_names):
    """ This function returns the agents information like name, car color, display name
    Arguments:
        racecar_names (list): comma seperated racecar names
    Returns:
        (list): Racecar information such as name, car color, display name
    """
    racecars = racecar_names
    racecars_info = list()
    racecars_color = force_list(rospy.get_param("CAR_COLOR", DEFAULT_COLOR))
    racecars_display_name = get_video_display_name()

    for i, racecar_name in enumerate(racecars):
        racecar_dict = dict()
        racecar_dict['name'] = racecar_name
        racecar_dict['racecar_color'] = RaceCarColorToRGB[racecars_color[i]].value
        racecar_dict['display_name'] = racecars_display_name[i]
        racecars_info.append(racecar_dict)
    return racecars_info

def main(racecar_names):
    """ Main function for kinesis_video_camera
    Arguments:
        racecar_names (list): racecar_names as a comma seperated string
    """
    try:
        racecars_info = get_racecars_info(racecar_names)
        for racecar in racecars_info:
            # Instantiate KinesisVideoCamera objects for each racecar
            KinesisVideoCamera(racecar['name'], racecars_info)
    except Exception as err_msg:
        log_and_exit("Exception in Kinesis Video camera ros node: {}"
                         .format(err_msg),
                     SIMAPP_SIMULATION_KINESIS_VIDEO_CAMERA_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)

if __name__ == '__main__':
    # comma seperated racecar names passed as an argument to the node
    rospy.init_node('kinesis_video_camera_node', anonymous=True)
    RACER_NUM = int(sys.argv[1])
    racecar_names = get_racecar_names(RACER_NUM)
    main(racecar_names)
    rospy.spin()
