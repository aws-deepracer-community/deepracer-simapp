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
from queue import Queue
import cv2
import rospy
from std_srvs.srv import Empty
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ROSImg
from collections import OrderedDict

from markov.utils import DoubleBuffer, get_racecar_names
from markov.constants import DEFAULT_COLOR
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_KINESIS_VIDEO_CAMERA_EXCEPTION)
from markov.reset.constants import (RaceType)
from markov.rospy_wrappers import ServiceProxyWrapper
from markov.utils import get_racecar_idx
from markov.virtual_event.constants import WAIT_DISPLAY_NAME
from deepracer_simulation_environment.srv import (VideoMetricsSrvRequest,
                                                  VideoMetricsSrv,
                                                  VirtualEventVideoEditSrv,
                                                  VirtualEventVideoEditSrvResponse)
from mp4_saving.constants import (RaceCarColorToRGB, CameraTypeParams,
                                  Mp4Parameter, FrameQueueData, MAX_FRAMES_IN_QUEUE,
                                  KVS_PUBLISH_PERIOD, QUEUE_WAIT_TIME,
                                  VirtualEventData)
from mp4_saving.virtual_event_single_agent_image_editing import VirtualEventSingleAgentImageEditing
from mp4_saving.virtual_event_multi_agent_image_editing import VirtualEventMultiAgentImageEditing
from mp4_saving import utils
from mp4_saving.save_to_mp4 import SaveToMp4
from mp4_saving.virtual_event_video_metrics import VirtualEventVideoMetrics

LOG = Logger(__name__, logging.INFO).get_logger()


class VirtualEventVideoEditor(object):
    """ This node is used to produce frames for the AWS kinesis video stream and
    for saving the mp4 and uploading to S3. Both are subscribed to the output of
    the image topic produced by this node.
    """
    _agents_metrics = list()
    _virtual_event_video_metrics = VirtualEventVideoMetrics()
    _racecars_info = list()

    def __init__(self, racecar_name, agent_name):
        #
        # We have no guarantees as to when gazebo will load the model, therefore we need
        # to wait until the model is loaded and markov packages has spawned all the models
        #
        rospy.wait_for_service('/robomaker_markov_package_ready')
        self._agents_metrics.append(DoubleBuffer(clear_data_on_get=False))

        self.racecar_name = racecar_name
        self.agent_name = agent_name
        racecar_index = get_racecar_idx(racecar_name)
        self.racecar_index = racecar_index if racecar_index else 0

        # init cv bridge
        self.bridge = CvBridge()

        # hard code because virtual event is eval only.
        self.is_training = False

        racecar_dict = dict()
        racecar_dict['name'] = racecar_name
        racecar_dict['racecar_color'] = RaceCarColorToRGB[DEFAULT_COLOR].value
        racecar_dict['display_name'] = WAIT_DISPLAY_NAME
        self._racecars_info.append(racecar_dict)

        self.job_type_image_edit = self._get_image_editing_job_type()

        # Fetching main camera frames, start consumer thread and producer thread for main camera frame
        main_camera_topic = "/{}/{}/zed/rgb/image_rect_color".format(self.racecar_name, "main_camera")

        # All KVS related initialization
        self._kvs_frame_buffer = DoubleBuffer(clear_data_on_get=False)

        # Publish to KVS stream topic
        self.kvs_pub = rospy.Publisher('/{}/deepracer/kvs_stream'.format(self.racecar_name), ROSImg, queue_size=1)

        # All Mp4 related initialization
        self._mp4_queue = Queue()
        self._mp4_edited_frame_queue = Queue(MAX_FRAMES_IN_QUEUE)
        self._mp4_condition_lock = Condition()

        # Initialize save mp4 ROS service for the markov package to signal when to
        # start and stop collecting video frames
        race_type = rospy.get_param("RACE_TYPE", RaceType.TIME_TRIAL.value)
        is_f1_race_type = race_type == RaceType.F1.value
        camera_info = utils.get_cameratype_params(self.racecar_name, self.agent_name, is_f1_race_type)
        self.save_to_mp4_obj = SaveToMp4(camera_infos=[camera_info[CameraTypeParams.CAMERA_PIP_PARAMS],
                                                       camera_info[CameraTypeParams.CAMERA_45DEGREE_PARAMS],
                                                       camera_info[CameraTypeParams.CAMERA_TOPVIEW_PARAMS]],
                                         fourcc=Mp4Parameter.FOURCC.value,
                                         fps=Mp4Parameter.FPS.value,
                                         frame_size=Mp4Parameter.FRAME_SIZE.value)
        rospy.Service('/{}/save_mp4/subscribe_to_save_mp4'.format(self.racecar_name),
                      VirtualEventVideoEditSrv, self.subscribe_to_save_mp4)
        rospy.Service('/{}/save_mp4/unsubscribe_from_save_mp4'.format(self.racecar_name),
                      Empty, self.unsubscribe_to_save_mp4)

        # Publish to save mp4 topic
        self.mp4_main_camera_pub = rospy.Publisher('/{}/deepracer/main_camera_stream'.format(self.racecar_name), ROSImg,
                                                   queue_size=1)

        # ROS service to get video metrics
        rospy.wait_for_service("/{}/{}".format(self.agent_name, "mp4_video_metrics"))
        self.mp4_video_metrics_srv = ServiceProxyWrapper("/{}/{}".format(self.agent_name, "mp4_video_metrics"),
                                                         VideoMetricsSrv, persistent=True)
        self.is_save_mp4_enabled = False

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
        self._virtual_event_video_metrics.reset()
        self.is_save_mp4_enabled = True
        self._racecars_info[self.racecar_index]['display_name'] = req.display_name
        self.job_type_image_edit = self._get_image_editing_job_type()
        self.save_to_mp4_obj.subscribe_to_save_mp4()
        return VirtualEventVideoEditSrvResponse(success=True)

    def unsubscribe_to_save_mp4(self, req):
        """ Ros service handler function used to unsubscribe from the Image topic.
        This will take care of cleaning and releasing the cv2 VideoWriter
        Arguments:
            req (req): Dummy req else the ros service throws exception
        Return:
            [] - Empty list else ros service throws exception
        """
        self.is_save_mp4_enabled = False
        # This is required because when unsubscribe call is made the frames in the queue will continue editing,
        # but at this time the 45degree camera will continue to be subscribed and saved to mp4 which we do not want.
        # The de facto behavior for virtual event is that we flush everything post empty the queue
        # so keeping it here.
        camera_topics_stop_post_empty_queue = [CameraTypeParams.CAMERA_45DEGREE_PARAMS.value,
                                               CameraTypeParams.CAMERA_TOPVIEW_PARAMS.value,
                                               CameraTypeParams.CAMERA_PIP_PARAMS.value]
        while not (self._mp4_queue.empty() and self._mp4_edited_frame_queue.empty()):
            time.sleep(1)
        self.save_to_mp4_obj.unsubscribe_to_save_mp4(camera_topics_stop_post_empty_queue)
        return []

    def _get_image_editing_job_type(self):
        """ This determines what kinding of image editing should be done based on the race type
        Returns:
            Union(VirtualEventSingleAgentImageEditing,
                  VirtualEventMultiAgentImageEditing): VirtualEventSingleAgentImageEditing instance
        """
        race_type = rospy.get_param("RACE_TYPE", RaceType.TIME_TRIAL.value)
        if race_type in [RaceType.TIME_TRIAL.value, RaceType.OBJECT_AVOIDANCE.value,
                         RaceType.HEAD_TO_BOT.value]:
            return VirtualEventSingleAgentImageEditing(self.racecar_name, self._racecars_info, race_type)
        elif race_type == RaceType.HEAD_TO_MODEL.value:
            return VirtualEventMultiAgentImageEditing(self.racecar_name, self._racecars_info, race_type)
        else:
            raise Exception("[Virtual Event]: Unknown job type for image editing")

    def _update_racers_metrics(self):
        """ Used to update the racers metric information
        """
        if not rospy.is_shutdown():
            video_metrics = self.mp4_video_metrics_srv(VideoMetricsSrvRequest())
            self._agents_metrics[self.racecar_index].put(video_metrics)

    def _edit_camera_images(self, frame_data):
        """ Edit camera image by calling respective job type

        Arguments:
            frame_data (dict): Dictionary of frame, agent_metric_info, training_phase

        Returns:
            Image: Edited image
        """
        frame = frame_data[FrameQueueData.FRAME.value]
        metric_info = {
            FrameQueueData.AGENT_METRIC_INFO.value: frame_data[FrameQueueData.AGENT_METRIC_INFO.value],
            FrameQueueData.VIRTUAL_EVENT_INFO.value: frame_data[FrameQueueData.VIRTUAL_EVENT_INFO.value],
            FrameQueueData.TRAINING_PHASE.value: frame_data[FrameQueueData.TRAINING_PHASE.value]
        }

        # convert ros image message to cv image
        try:
            major_cv_image = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        except CvBridgeError as ex:
            LOG.info("[Virtual Event]: ROS image message to cv2 error: {}".format(ex))

        major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2RGBA)

        # Edit the image based on the racecar type and job type
        major_cv_image = self.job_type_image_edit.edit_image(major_cv_image, metric_info)

        # convert cv image back to ros image message
        try:
            edited_frame = self.bridge.cv2_to_imgmsg(major_cv_image, "bgr8")
        except CvBridgeError as ex:
            LOG.info("[Virtual Event]: cv2 to ROS image message error: {}".format(ex))
        return edited_frame

    def _producer_frame_thread(self, frame):
        """ Callback for the main camera frame. Once a new image is received, all the required
        service calls are made to get the video metric information. Then for Mp4 its put into the queue
        but for the KVS its put into a double since we only care for the latest image for KVS

        Arguments:
            frame (cv2.ImgMsg): Image/Sensor topic of the camera image frame
        """
        if not rospy.is_shutdown():
            self._update_racers_metrics()
            # Get frame from main camera & agents metric information
            agent_metric_info = [metrics.get() for metrics in self._agents_metrics]

            racer_metrics = OrderedDict()
            for idx, metric in enumerate(agent_metric_info):
                if self._racecars_info[idx]['display_name'] == WAIT_DISPLAY_NAME:
                    racer_metrics = OrderedDict()
                    break
                racer_metrics[self._racecars_info[idx]['display_name']] = metric
            self._virtual_event_video_metrics.update(racer_metrics)
            queue_data = {
                FrameQueueData.FRAME.value: frame,
                FrameQueueData.AGENT_METRIC_INFO.value: agent_metric_info,
                FrameQueueData.TRAINING_PHASE.value: '',
                FrameQueueData.VIRTUAL_EVENT_INFO.value: {
                    VirtualEventData.SIM_TIME.value: self._virtual_event_video_metrics.sim_time,
                    VirtualEventData.TIME_TO_LEADER.value: OrderedDict(self._virtual_event_video_metrics.time_to_leader),
                    VirtualEventData.LAP.value: self._virtual_event_video_metrics.lap}
            }

            self._kvs_frame_buffer.put(queue_data)

            if self.is_save_mp4_enabled:
                with self._mp4_condition_lock:
                    if self._mp4_queue.qsize() == MAX_FRAMES_IN_QUEUE:
                        LOG.info("[Virtual Event]: Dropping Mp4 frame from the queue")
                        self._mp4_queue.get()
                    # Append to the MP4 queue
                    self._mp4_queue.put(queue_data)
                    self._mp4_condition_lock.notify()

    def _consumer_mp4_frame_thread(self):
        """ Consumes the frame produced by the _producer_frame_thread and edits the image
        The edited image is put into another queue for publishing to MP4 topic
        """
        while not rospy.is_shutdown():
            with self._mp4_condition_lock:
                if self._mp4_queue.empty():
                    # Wait untill producer adds something to queue
                    self._mp4_condition_lock.wait(QUEUE_WAIT_TIME)
                    continue
                # Pop from the queue and edit the image
                frame_data = self._mp4_queue.get()
                if len(self._racecars_info) > len(frame_data[FrameQueueData.AGENT_METRIC_INFO.value]):
                    # Waiting for all the agents to initialize before editing videos
                    # There could be condition when racecar_0 starts editing frames before racecar_1 is initialized
                    time.sleep(KVS_PUBLISH_PERIOD)
                    continue
                elif len(self._racecars_info) < len(frame_data[FrameQueueData.AGENT_METRIC_INFO.value]):
                    log_and_exit("Agents video editing metric cannot be larger than racecar info",
                                 SIMAPP_SIMULATION_KINESIS_VIDEO_CAMERA_EXCEPTION,
                                 SIMAPP_EVENT_ERROR_CODE_500)
                edited_frame = self._edit_camera_images(frame_data)
                self._mp4_edited_frame_queue.put(edited_frame)

    def _mp4_publisher(self):
        """ Publishing the latest edited image to Mp4 topic at KVS_PUBLISH_PERIOD
        """
        try:
            prev_time = time.time()
            while not rospy.is_shutdown():
                if not self._mp4_edited_frame_queue.empty():
                    if rospy.is_shutdown():
                        break
                    self.mp4_main_camera_pub.publish(self._mp4_edited_frame_queue.get())
                cur_time = time.time()
                time_diff = cur_time - prev_time
                time.sleep(max(KVS_PUBLISH_PERIOD - time_diff, 0))
                prev_time = time.time()
        except (rospy.ROSInterruptException, rospy.ROSException):
            pass

    def _kvs_publisher(self):
        """ Publishing the latest edited image to KVS topic at KVS_PUBLISH_PERIOD
        """
        try:
            prev_time = time.time()
            while not rospy.is_shutdown():
                frame_data = self._kvs_frame_buffer.get()
                if len(self._racecars_info) > len(frame_data[FrameQueueData.AGENT_METRIC_INFO.value]):
                    # Waiting for all the agents to initialize before editing videos
                    # There could be condition when racecar_0 starts editing frames before racecar_1 is initialized
                    time.sleep(KVS_PUBLISH_PERIOD)
                    continue
                elif len(self._racecars_info) < len(frame_data[FrameQueueData.AGENT_METRIC_INFO.value]):
                    log_and_exit("Agents video editing metric cannot be larger than racecar info",
                                 SIMAPP_SIMULATION_KINESIS_VIDEO_CAMERA_EXCEPTION,
                                 SIMAPP_EVENT_ERROR_CODE_500)
                edited_frame = self._edit_camera_images(frame_data)
                if not rospy.is_shutdown():
                    self.kvs_pub.publish(edited_frame)
                    cur_time = time.time()
                    time_diff = cur_time - prev_time
                    time.sleep(max(KVS_PUBLISH_PERIOD - time_diff, 0))
                    prev_time = time.time()
        except (rospy.ROSInterruptException, rospy.ROSException):
            pass


def main():
    """ Main function for virtual event video editor
    """
    try:
        racer_num = int(sys.argv[1])
        racecar_names = get_racecar_names(racer_num)
        for racecar_name in racecar_names:
            VirtualEventVideoEditor(
                racecar_name=racecar_name,
                agent_name=racecar_name.replace("racecar", "agent"))
    except Exception as err_msg:
        log_and_exit("[Virtual Event]: Exception in Kinesis Video camera ros node: {}".format(err_msg),
                     SIMAPP_SIMULATION_KINESIS_VIDEO_CAMERA_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)


if __name__ == '__main__':
    rospy.init_node('virtual_event_video_editor_node', anonymous=True)
    main()
    rospy.spin()
