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
from threading import Thread, Condition
from Queue import Queue
import cv2
import rospy
from std_srvs.srv import Empty
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ROSImg
from std_msgs.msg import String

from markov.utils import DoubleBuffer, force_list, get_video_display_name, get_racecar_names
from markov.constants import DEFAULT_COLOR                     
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_KINESIS_VIDEO_CAMERA_EXCEPTION)
from markov.reset.constants import (RaceType)
from markov.rospy_wrappers import ServiceProxyWrapper
from markov.utils import get_racecar_idx
from deepracer_simulation_environment.srv import VideoMetricsSrvRequest, VideoMetricsSrv
from mp4_saving.constants import (RaceCarColorToRGB, CameraTypeParams,
                                  Mp4Parameter, FrameQueueData, MAX_FRAMES_IN_QUEUE,
                                  KVS_PUBLISH_PERIOD, QUEUE_WAIT_TIME, FrameTypes)
from mp4_saving.single_agent_image_editing import SingleAgentImageEditing
from mp4_saving.multi_agent_image_editing import MultiAgentImageEditing
from mp4_saving.training_image_editing import TrainingImageEditing
from mp4_saving.f1_image_editing import F1ImageEditing
from mp4_saving import utils
from mp4_saving.save_to_mp4 import SaveToMp4

LOG = Logger(__name__, logging.INFO).get_logger()

class AgentsVideoEditor(object):
    """ This node is used to produce frames for the AWS kinesis video stream and
    for saving the mp4 and uploading to S3. Both are subscribed to the output of
    the image topic produced by this node.
    """
    _agents_metrics = list()
    _mp4_queue = list()
    _mp4_edited_frame_queue = list()
    _mp4_condition_lock = list()

    def __init__(self, racecar_name, racecars_info):
        #
        # We have no guarantees as to when gazebo will load the model, therefore we need
        # to wait until the model is loaded and markov packages has spawned all the models
        #
        rospy.wait_for_service('/robomaker_markov_package_ready')

        self._agents_metrics.append(DoubleBuffer(clear_data_on_get=False))

        self.racecar_name = racecar_name
        self.racecars_info = racecars_info
        racecar_index = get_racecar_idx(racecar_name)
        self.racecar_index = racecar_index if racecar_index else 0
        self.agent_name = utils.racecar_name_to_agent_name(racecars_info, racecar_name)

        # init cv bridge
        self.bridge = CvBridge()

        # This determines what kind of image editing should be done based on the race type
        self.is_training = rospy.get_param("JOB_TYPE") == 'TRAINING'
        self.race_type = rospy.get_param("RACE_TYPE", RaceType.TIME_TRIAL.value)
        self.is_f1_race_type = self.race_type == RaceType.F1.value
        self.job_type_image_edit = self._get_image_editing_job_type()
        if self.is_training:
            # String indicating the current phase
            self._current_training_phase = DoubleBuffer(clear_data_on_get=False)
            self._current_training_phase.put('Initializing')
            # Subscriber to get the phase of the training (Ideal, training, evaluation)
            rospy.Subscriber('/agent/training_phase', String, self._training_phase_cb)

        # Fetching main camera frames, start consumer thread and producer thread for main camera frame
        main_camera_topic = "/{}/{}/zed/rgb/image_rect_color".format(self.racecar_name, "main_camera")

        # All KVS related initialization
        self._kvs_frame_buffer = DoubleBuffer(clear_data_on_get=False)

        # Publish to KVS stream topic
        self.kvs_pub = rospy.Publisher('/{}/deepracer/kvs_stream'.format(self.racecar_name), ROSImg, queue_size=1)

        # All Mp4 related initialization
        self._mp4_queue.append(Queue())
        self._mp4_edited_frame_queue.append(Queue(MAX_FRAMES_IN_QUEUE))
        self._mp4_condition_lock.append(Condition())

        # Initialize save mp4 ROS service for the markov package to signal when to
        # start and stop collecting video frames
        camera_info = utils.get_cameratype_params(self.racecar_name, self.agent_name, self.is_f1_race_type)
        self.save_to_mp4_obj = SaveToMp4(camera_infos=[camera_info[CameraTypeParams.CAMERA_PIP_PARAMS],
                                                       camera_info[CameraTypeParams.CAMERA_45DEGREE_PARAMS],
                                                       camera_info[CameraTypeParams.CAMERA_TOPVIEW_PARAMS]],
                                         fourcc=Mp4Parameter.FOURCC.value,
                                         fps=Mp4Parameter.FPS.value,
                                         frame_size=Mp4Parameter.FRAME_SIZE.value)
        rospy.Service('/{}/save_mp4/subscribe_to_save_mp4'.format(self.racecar_name),
                      Empty, self.subscribe_to_save_mp4)
        rospy.Service('/{}/save_mp4/unsubscribe_from_save_mp4'.format(self.racecar_name),
                      Empty, self.unsubscribe_to_save_mp4)

        # Publish to save mp4 topic
        self.mp4_main_camera_pub = rospy.Publisher('/{}/deepracer/main_camera_stream'.format(self.racecar_name), ROSImg,
                                                   queue_size=1)

        # ROS service to get video metrics
        rospy.wait_for_service("/{}/{}".format(self.agent_name, "mp4_video_metrics"))
        self.mp4_video_metrics_srv = ServiceProxyWrapper("/{}/{}".format(self.agent_name, "mp4_video_metrics"),
                                                         VideoMetricsSrv)
        self.is_save_mp4_enabled = False

        # Only F1 race requires top camera frames edited
        if self.is_f1_race_type:
            self._top_camera_frame_buffer = DoubleBuffer(clear_data_on_get=False)
            top_camera_topic = "/sub_camera/zed/rgb/image_rect_color"
            rospy.Subscriber(top_camera_topic, ROSImg, self._top_camera_cb)
            self.top_camera_mp4_pub = rospy.Publisher('/{}/topcamera/deepracer/mp4_stream'.format(racecar_name),
                                                      ROSImg, queue_size=1)

        rospy.Subscriber(main_camera_topic, ROSImg, self._producer_frame_thread)
        Thread(target=self._consumer_mp4_frame_thread).start()
        Thread(target=self._kvs_publisher).start()
        Thread(target=self._mp4_publisher).start()

    def subscribe_to_save_mp4(self, req):
        """ Ros service handler function used to subscribe to the Image topic.
        Arguments:
            req (req): Dummy req else the ros service throws exception
        Return:
            [] - Empty list else ros service throws exception
        """
        self.is_save_mp4_enabled = True
        self.save_to_mp4_obj.subscribe_to_save_mp4()
        return []

    def unsubscribe_to_save_mp4(self, req):
        """ Ros service handler function used to unsubscribe from the Image topic.
        This will take care of cleaning and releasing the cv2 VideoWriter
        Arguments:
            req (req): Dummy req else the ros service throws exception
        Return:
            [] - Empty list else ros service throws exception
        """
        self.is_save_mp4_enabled = False
        while not (self._mp4_queue[self.racecar_index].empty() and self._mp4_edited_frame_queue[self.racecar_index].empty()):
            time.sleep(1)
        self.save_to_mp4_obj.unsubscribe_to_save_mp4()
        return []

    def _top_camera_cb(self, frame):
        '''Callback for the frames being publish by the top camera topic
            frame - Frames, of type Image, being published by main camera topic
        '''
        self._top_camera_frame_buffer.put(frame)

    def _training_phase_cb(self, phase):
        """ Callback function that gives the training phase - Whether its in
        evaluation, ideal, training, initializing
        Args:
            phase: [description]
        """
        self._current_training_phase.put(phase.data)

    def _get_image_editing_job_type(self):
        """ This determines what kinding of image editing should be done based on the race type
        Returns:
            ImageEditingObj: Instantiating an object based on training/evaluation and racetype
        """
        if self.is_training:
            return TrainingImageEditing(self.racecar_name, self.racecars_info, self.race_type)
        elif self.is_f1_race_type:
            return F1ImageEditing(self.racecar_name, self.racecars_info, self.race_type)
        elif self.race_type == RaceType.HEAD_TO_MODEL.value:
            return MultiAgentImageEditing(self.racecar_name, self.racecars_info,
                                          self.race_type)
        elif self.race_type in [RaceType.TIME_TRIAL.value, RaceType.OBJECT_AVOIDANCE.value,
                                RaceType.HEAD_TO_BOT.value]:
            return SingleAgentImageEditing(self.racecar_name, self.racecars_info, self.race_type)
        raise Exception("Unknown job type for image editing")

    def _update_racers_metrics(self):
        """ Used to update the racers metric information
        """
        if not rospy.is_shutdown():
            video_metrics = self.mp4_video_metrics_srv(VideoMetricsSrvRequest())
            self._agents_metrics[self.racecar_index].put(video_metrics)

    def _edit_camera_images(self, frame_data, is_mp4):
        """ Edit camera image by calling respective job type

        Arguments:
            frame_data (dict): Dictionary of frame, agent_metric_info, training_phase
            is_mp4 (bool): Is this edit camera image for kvs or mp4

        Returns:
            Image: Edited image
        """
        main_frame = frame_data[FrameQueueData.FRAME.value][FrameTypes.MAIN_CAMERA_FRAME.value]
        metric_info = {
            FrameQueueData.AGENT_METRIC_INFO.value: frame_data[FrameQueueData.AGENT_METRIC_INFO.value],
            FrameQueueData.TRAINING_PHASE.value: frame_data[FrameQueueData.TRAINING_PHASE.value]
        }

        # convert ros image message to cv image
        try:
            major_cv_image = self.bridge.imgmsg_to_cv2(main_frame, "bgr8")
            major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2RGBA)
            # Edit the image based on the racecar type and job type
            major_cv_image = self.job_type_image_edit.edit_image(major_cv_image, metric_info)
            edited_main_frame = self.bridge.cv2_to_imgmsg(major_cv_image, "bgr8")

            # Edit top camera image only if its F1
            edited_top_frame = []
            if self.is_f1_race_type and is_mp4:
                top_camera_frame = frame_data[FrameQueueData.FRAME.value][FrameTypes.TOP_CAMERA_FRAME.value]
                top_cv_image = self.bridge.imgmsg_to_cv2(top_camera_frame, "bgr8")
                top_cv_image = cv2.cvtColor(top_cv_image, cv2.COLOR_RGB2RGBA)
                top_cv_image = self.job_type_image_edit.edit_top_camera_image(top_cv_image, metric_info)
                edited_top_frame = self.bridge.cv2_to_imgmsg(top_cv_image, "bgr8")
            return {
                FrameTypes.MAIN_CAMERA_FRAME.value: edited_main_frame,
                FrameTypes.TOP_CAMERA_FRAME.value: edited_top_frame
            }
        except CvBridgeError as ex:
            LOG.info("cv2 to ROS image message error: {}".format(ex))

    def _producer_frame_thread(self, frame):
        """ Callback for the main camera frame. Once a new image is received, all the required
        service calls are made to get the video metric information. Then for Mp4 its put into the queue
        but for the KVS its put into a double since we only care for the latest image for KVS

        Arguments:
            frame (cv2.ImgMsg): Image/Sensor topic of the camera image frame
        """
        if not rospy.is_shutdown():
            if len(self.racecars_info) != len(self._mp4_queue):
                pass
            self._update_racers_metrics()
            # Get frame from main camera & agents metric information
            agent_metric_info = [metrics.get() for metrics in self._agents_metrics]
            queue_data = {
                FrameQueueData.FRAME.value: {
                    FrameTypes.MAIN_CAMERA_FRAME.value: frame,
                    FrameTypes.TOP_CAMERA_FRAME.value: self._top_camera_frame_buffer.get() if self.is_f1_race_type else []
                },
                FrameQueueData.AGENT_METRIC_INFO.value: agent_metric_info,
                FrameQueueData.TRAINING_PHASE.value: self._current_training_phase.get() if self.is_training else ''
            }

            self._kvs_frame_buffer.put(queue_data)

            if self.is_save_mp4_enabled:
                with self._mp4_condition_lock[self.racecar_index]:
                    if self._mp4_queue[self.racecar_index].qsize() == MAX_FRAMES_IN_QUEUE:
                        LOG.info("Dropping Mp4 frame from the queue")
                        self._mp4_queue[self.racecar_index].get()
                    # Append to the MP4 queue
                    self._mp4_queue[self.racecar_index].put(queue_data)
                    self._mp4_condition_lock[self.racecar_index].notify()

    def _consumer_mp4_frame_thread(self):
        """ Consumes the frame produced by the _producer_frame_thread and edits the image
        The edited image is put into another queue for publishing to MP4 topic
        """
        while not rospy.is_shutdown():
            with self._mp4_condition_lock[self.racecar_index]:
                if self._mp4_queue[self.racecar_index].empty():
                    # Wait untill producer adds something to queue
                    self._mp4_condition_lock[self.racecar_index].wait(QUEUE_WAIT_TIME)
                    continue
                # Pop from the queue and edit the image
                frame_data = self._mp4_queue[self.racecar_index].get()
                edited_frames = self._edit_camera_images(frame_data, is_mp4=True)
                self._mp4_edited_frame_queue[self.racecar_index].put(edited_frames)

    def _mp4_publisher(self):
        """ Publishing the latest edited image to Mp4 topic at 15FPS simulation time

        In case of mp4 videos we want to maintain the 15FPS no matter what RTF the actual
        simulator is running. This is done by using the rospy.Rate. The 15FPS will be the simulation time
        and not the real time.
        """
        try:
            publish_rate = rospy.Rate(Mp4Parameter.FPS.value)
            while not rospy.is_shutdown():
                if not self._mp4_edited_frame_queue[self.racecar_index].empty():
                    if rospy.is_shutdown():
                        break
                    edited_frames = self._mp4_edited_frame_queue[self.racecar_index].get()
                    self.mp4_main_camera_pub.publish(edited_frames[FrameTypes.MAIN_CAMERA_FRAME.value])
                    if self.is_f1_race_type:
                        self.top_camera_mp4_pub.publish(edited_frames[FrameTypes.TOP_CAMERA_FRAME.value])
                publish_rate.sleep()
        except (rospy.ROSInterruptException, rospy.ROSException):
            pass

    def _kvs_publisher(self):
        """ Publishing the latest edited image to KVS topic at 15 FPS real time.

        In case of Kinesis video stream we want to publish frames real time at 15 FPS. If the frames
        are not published at this rate, there will be jitter and video will be laggy. So it has to always
        be the real time. Unlike mp4_publisher this cannot be a simulation time.
        """
        try:
            prev_time = time.time()
            while not rospy.is_shutdown():
                frame_data = self._kvs_frame_buffer.get()
                edited_frames = self._edit_camera_images(frame_data, is_mp4=False)
                if not rospy.is_shutdown():
                    self.kvs_pub.publish(edited_frames[FrameTypes.MAIN_CAMERA_FRAME.value])
                    cur_time = time.time()
                    time_diff = cur_time - prev_time
                    time.sleep(max(KVS_PUBLISH_PERIOD - time_diff, 0))
                    prev_time = time.time()
        except (rospy.ROSInterruptException, rospy.ROSException):
            pass


def get_racecars_info(racecar_names):
    """ This function returns the agents information like name, car color, display name
    Arguments:
        racecar_names (list): comma seperated racecar names
    Returns:
        (list): Racecar information such as name, car color, display name
    """
    racecars = racecar_names
    racecars_info = list()
    racecars_color = force_list(rospy.get_param("CAR_COLOR",
                                                [DEFAULT_COLOR] * len(racecar_names)))
    racecars_display_name = get_video_display_name()

    for i, racecar_name in enumerate(racecars):
        racecar_dict = dict()
        racecar_dict['name'] = racecar_name
        racecar_dict['racecar_color'] = racecars_color[i]
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
            # Instantiate AgentCameraVideoEditor objects for each racecar
            AgentsVideoEditor(racecar['name'], racecars_info)
    except Exception as err_msg:
        log_and_exit("Exception in Kinesis Video camera ros node: {}".format(err_msg),
                     SIMAPP_SIMULATION_KINESIS_VIDEO_CAMERA_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)


if __name__ == '__main__':
    # comma seperated racecar names passed as an argument to the node
    rospy.init_node('agent_camera_video_editor_node', anonymous=True)
    RACER_NUM = int(sys.argv[1])
    RACECAR_NAMES = get_racecar_names(RACER_NUM)
    main(RACECAR_NAMES)
    rospy.spin()
