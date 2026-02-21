#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

import os
import sys
import time
import logging
from threading import Thread, Lock
import queue
import cv2
import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
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
from markov.rclpy_wrappers import ServiceProxyWrapper 
from markov.utils import get_racecar_idx
from markov.world_config import WorldConfig
from deepracer_simulation_environment.srv import VideoMetricsSrv
from mp4_saving.constants import (CameraTypeParams,
                                  Mp4Parameter, FrameQueueData, MAX_FRAMES_IN_QUEUE,
                                  KVS_PUBLISH_PERIOD, QUEUE_WAIT_TIME, FrameTypes,
                                  METRICS_POLL_PERIOD_SEC)
from mp4_saving.single_agent_image_editing import SingleAgentImageEditing
from mp4_saving.multi_agent_image_editing import MultiAgentImageEditing
from mp4_saving.training_image_editing import TrainingImageEditing
from mp4_saving.f1_image_editing import F1ImageEditing
from mp4_saving import utils
from mp4_saving.save_to_mp4 import SaveToMp4

LOG = Logger(__name__, logging.INFO).get_logger()

# QoS profile matching bridge publisher
CAMERA_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class AgentsVideoEditor(Node):
    """ This node is used to produce frames for the AWS kinesis video stream and
    for saving the mp4 and uploading to S3. Both are subscribed to the output of
    the image topic produced by this node.
    """
    _agents_metrics = list()
    _mp4_queue = list()

    def __init__(self, racecar_name, racecars_info, is_publish_to_kvs_stream):
        LOG.debug("DEBUG VideoEditor: __init__ starting for racecar_name=%s", racecar_name)
        
        try:
            super().__init__(f'agents_video_editor_{racecar_name}')
        except Exception as e:
            LOG.error("Error initializing ROS2 node: %s", e)
            raise

        # ROS2 ARCHITECTURE FIX: Set basic attributes immediately (like ROS1)
        # ServiceProxyWrapper handles service coordination with retries, no upfront waiting needed
        try:
            self._agent_metrics_buffer = DoubleBuffer(clear_data_on_get=False)
            self._agents_metrics.append(self._agent_metrics_buffer)
            self.racecar_name = racecar_name
            self.racecars_info = racecars_info
            self._latest_agent_metric_info = [None] * len(self.racecars_info)
            racecar_index = get_racecar_idx(racecar_name)
            self.racecar_index = racecar_index if racecar_index else 0
            self.agent_name = utils.racecar_name_to_agent_name(self.racecars_info, racecar_name)
            self._is_publish_to_kvs_stream = is_publish_to_kvs_stream
        except Exception as e:
            LOG.error("Error setting basic attributes: %s", e)
            raise

        # init cv bridge
        self.bridge = CvBridge()
        self._shutdown_requested = False
        self._metrics_call_lock = Lock()

        # ROS2: Declare parameters
        try:
            self.declare_parameter('JOB_TYPE', WorldConfig.get_param('JOB_TYPE', 'EVALUATION'))
            self.declare_parameter('RACE_TYPE', WorldConfig.get_param('RACE_TYPE', RaceType.TIME_TRIAL.value))
            self.is_training = self.get_parameter('JOB_TYPE').get_parameter_value().string_value == 'TRAINING'
            self.race_type = self.get_parameter('RACE_TYPE').get_parameter_value().string_value
            self.is_f1_race_type = self.race_type == RaceType.F1.value
        except Exception as e:
            LOG.error("Error setting parameters: %s", e)
            raise

        # Create job type after all attributes are set
        try:
            self.job_type_image_edit = self._get_image_editing_job_type()
        except Exception as e:
            LOG.error("Error creating job_type_image_edit: %s", e)
            raise

        # agents_metrics already initialized in __init__, no need to append again
        LOG.debug("DEBUG VideoEditor: agents_metrics already initialized with length %s", len(self._agents_metrics))

        try:
            self.racecar_name = racecar_name
            self.racecars_info = racecars_info
            racecar_index = get_racecar_idx(racecar_name)
            self.racecar_index = racecar_index if racecar_index else 0
        except Exception as e:
            LOG.error("Error setting racecar info: %s", e)
            raise

        try:
            self.agent_name = utils.racecar_name_to_agent_name(self.racecars_info, racecar_name)
            LOG.debug("VideoEditor: agent_name=%s", self.agent_name)
        except Exception as e:
            LOG.error("ERROR getting agent_name: %s", e)
            raise

        try:
            self._is_publish_to_kvs_stream = is_publish_to_kvs_stream
        except Exception as e:
            LOG.error("Error setting KVS stream flag: %s", e)
            raise
            
        if self.is_training:
            try:
                # String indicating the current phase
                self._current_training_phase = DoubleBuffer(clear_data_on_get=False)
                self._current_training_phase.put('Initializing')
                # Subscriber to get the phase of the training (Ideal, training, evaluation)
                self.training_phase_subscription = self.create_subscription(
                    String,
                    '/agent/training_phase',
                    self._training_phase_cb,
                    QoSProfile(depth=10)
                )
            except Exception as e:
                LOG.error("Error creating training phase subscription: %s", e)
                raise

        # Fetching main camera frames, start consumer thread and producer thread for main camera frame
        try:
            main_camera_topic = "/{}/{}/zed/rgb/image_rect_color".format(self.racecar_name, "main_camera")
        except Exception as e:
            LOG.error("Error creating main camera topic: %s", e)
            raise

        # All Mp4 related initialization
        try:
            self._agent_frame_queue = queue.Queue()
            self._agent_frame_queue_to_complete = 0
            self._agent_frame_queue_lock = Lock()
            self._mp4_queue.append(self._agent_frame_queue)
            LOG.debug("DEBUG VideoEditor: mp4_queue initialized")
        except Exception as e:
            LOG.error("Error initializing MP4 queue: %s", e)
            raise

        # Initialize save mp4 ROS service for the markov package to signal when to
        # start and stop collecting video frames
        try:
            camera_info = utils.get_cameratype_params(self.racecar_name, self.agent_name, self.is_f1_race_type)
        except Exception as e:
            LOG.error("Error in get_cameratype_params: %s", e)
            raise
        
        self.save_to_mp4_obj = SaveToMp4(camera_infos=[camera_info[CameraTypeParams.CAMERA_PIP_PARAMS],
                                                       camera_info[CameraTypeParams.CAMERA_45DEGREE_PARAMS],
                                                       camera_info[CameraTypeParams.CAMERA_TOPVIEW_PARAMS]],
                                         fourcc=Mp4Parameter.FOURCC.value,
                                         fps=Mp4Parameter.FPS.value,
                                         frame_size=Mp4Parameter.FRAME_SIZE.value)

        self._service_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self._metrics_timer_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        
        self.subscribe_service = self.create_service(
            Empty, 
            f'/{self.racecar_name}/save_mp4/subscribe_to_save_mp4',
            self.subscribe_to_save_mp4,
            callback_group=self._service_callback_group
        )
        
        self.unsubscribe_service = self.create_service(
            Empty,
            f'/{self.racecar_name}/save_mp4/unsubscribe_from_save_mp4', 
            self.unsubscribe_to_save_mp4,
            callback_group=self._service_callback_group
        )

        # Publish to save mp4 topic
        self.mp4_main_camera_pub = self.create_publisher(
            ROSImg,
            f'/{self.racecar_name}/deepracer/main_camera_stream',
            10
        )

        # ROS service to get video metrics
        # Increase retry attempts and timeout since the service may take time to become available
        self.mp4_video_metrics_srv = ServiceProxyWrapper("/{}/{}".format(self.agent_name, "mp4_video_metrics"),
                                                         VideoMetricsSrv,
                                                         max_retry_attempts=30,
                                                         timeout_sec=2.0,
                                                         wait_for_service=True)
        self._metrics_update_timer = self.create_timer(METRICS_POLL_PERIOD_SEC,
                                                        self._update_racers_metrics,
                                                        callback_group=self._metrics_timer_callback_group)
        self.is_save_mp4_enabled = False

        # Only F1 race requires top camera frames edited
        self.top_camera_mp4_pub = None
        if self.is_f1_race_type and self.racecar_index == 0:
            self._top_camera_frame_buffer = DoubleBuffer(clear_data_on_get=False)
            top_camera_topic = "/sub_camera/zed/rgb/image_rect_color"
            
            self.top_camera_subscription = self.create_subscription(
                ROSImg,
                top_camera_topic,
                self._top_camera_cb,
                CAMERA_QOS,
                callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
            )
            self.top_camera_mp4_pub = self.create_publisher(
                ROSImg,
                f'/{racecar_name}/topcamera/deepracer/mp4_stream',
                1
            )

        self._main_camera_frame_buffer = DoubleBuffer(clear_data_on_get=False)
        
        self.main_camera_subscription = self.create_subscription(
            ROSImg,
            main_camera_topic,
            self._producer_frame_thread,
            CAMERA_QOS,
            # Use callback group to separate camera callbacks from service callbacks
            # Reason: Fix ROS2 executor callback starvation - high-frequency camera callbacks 
            # monopolized executor thread, preventing service callbacks from being processed
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        )
        Thread(target=self._consumer_mp4_frame_thread).start()

        # Leaderboard jobs do not require KVS streams
        if self._is_publish_to_kvs_stream:
            # Buffer to store latest edited frame for KVS (avoids re-editing)
            self._kvs_edited_frame_buffer = DoubleBuffer(clear_data_on_get=False)
            # Publish to KVS stream topic
            self.kvs_pub = self.create_publisher(
                ROSImg,
                f'/{self.racecar_name}/deepracer/kvs_stream',
                1
            )
            Thread(target=self._kvs_publisher).start()

    def request_shutdown(self):
        self._shutdown_requested = True
        try:
            if hasattr(self, '_metrics_update_timer') and self._metrics_update_timer is not None:
                self._metrics_update_timer.cancel()
        except Exception:
            pass
        

    def subscribe_to_save_mp4(self, req, response):
        """ Ros service handler function used to subscribe to the Image topic.
        Arguments:
            req (req): Dummy req else the ros service throws exception
        Return:
            [] - Empty list else ros service throws exception
        """
        self.is_save_mp4_enabled = True
        self.save_to_mp4_obj.subscribe_to_save_mp4()
        return response

    def unsubscribe_to_save_mp4(self, req, response):
        """ Ros service handler function used to unsubscribe from the Image topic.
        This will take care of cleaning and releasing the cv2 VideoWriter
        Arguments:
            req (req): Dummy req else the ros service throws exception
        Return:
            [] - Empty list else ros service throws exception
        """
        while self._agent_frame_queue_lock.locked():
            LOG.info("Waiting for lock to be released for racecar_{}...".format(self.racecar_index))
            time.sleep(1.0)
        if self.is_save_mp4_enabled == False:
            LOG.info("Already unsubscribed from save_mp4 for racecar_{}. Ignoring duplicate unsubscribe call.".format(self.racecar_index))
            return response

        with self._agent_frame_queue_lock:
            self.is_save_mp4_enabled = False
            self._agent_frame_queue_to_complete = self._agent_frame_queue.qsize() + 5
            # This is required because when unsubscribe call is made the frames in the queue will continue editing,
            # but at this time the 45degree camera will continue to be subscribed and saved to mp4 which we do not want.
            camera_topics_stop_immediately, camera_topics_stop_post_empty_queue = list(), list()
            if not self.top_camera_mp4_pub:
                camera_topics_stop_immediately = [CameraTypeParams.CAMERA_45DEGREE_PARAMS.value,
                                                CameraTypeParams.CAMERA_TOPVIEW_PARAMS.value]
                camera_topics_stop_post_empty_queue = [CameraTypeParams.CAMERA_PIP_PARAMS.value]
            else:
                camera_topics_stop_immediately = [CameraTypeParams.CAMERA_45DEGREE_PARAMS.value]
                camera_topics_stop_post_empty_queue = [CameraTypeParams.CAMERA_TOPVIEW_PARAMS.value,
                                                    CameraTypeParams.CAMERA_PIP_PARAMS.value]

            self.save_to_mp4_obj.unsubscribe_to_save_mp4(camera_topics_stop_immediately)
            LOG.info("Waiting to flush the Mp4 queue for racecar_{}...".format(self.racecar_index))
            while not (self._agent_frame_queue.empty() or self._agent_frame_queue_to_complete == 0):
                LOG.info("Frames still in the queue for racecar_{}: {}".format(self.racecar_index, self._agent_frame_queue.qsize()))
                time.sleep(1)
            LOG.info("Done flushing the Mp4 queue for racecar_{}...".format(self.racecar_index))
            self.save_to_mp4_obj.unsubscribe_to_save_mp4(camera_topics_stop_post_empty_queue)
            return response

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
        if not rclpy.ok() or self._shutdown_requested:
            return
        if not self._metrics_call_lock.acquire(blocking=False):
            return
        try:
            video_metrics = self.mp4_video_metrics_srv(VideoMetricsSrv.Request())
            if not self._shutdown_requested:
                self._agent_metrics_buffer.put(video_metrics)
        except Exception:
            if not self._shutdown_requested and rclpy.ok():
                raise
        finally:
            self._metrics_call_lock.release()

    def _edit_main_camera_images(self, frame_data, metric_info, edited_frame_result):
        """ Thread to edit main camera frames

        Args:
            frame_data (dict): Dictionary of frame, agent_metric_info, training_phase
            metric_info (dict): This contains metric information to edit the videos also the phase like training phase
            edited_frame_result (dict): A mutable variable holding the dict result of edited frame
        """
        main_frame = frame_data[FrameQueueData.FRAME.value][FrameTypes.MAIN_CAMERA_FRAME.value]
        major_cv_image = self.bridge.imgmsg_to_cv2(main_frame, "bgr8")
        major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2RGBA)
        # Edit the image using single job type editor
        major_cv_image = self.job_type_image_edit.edit_image(major_cv_image, metric_info)
        edited_main_frame = self.bridge.cv2_to_imgmsg(major_cv_image, "bgr8")
        edited_frame_result[FrameTypes.MAIN_CAMERA_FRAME.value] = edited_main_frame

    def _edit_top_camera_images(self, frame_data, metric_info, edited_frame_result):
        """ Thread to edit top camera frames. This is only for the F1 format

        Args:
            frame_data (dict): Dictionary of frame, agent_metric_info, training_phase
            metric_info (dict): This contains metric information to edit the videos also the phase like training phase
            edited_frame_result (dict): A mutable variable holding the dict result of edited frame
        """
        top_camera_frame = frame_data[FrameQueueData.FRAME.value][FrameTypes.TOP_CAMERA_FRAME.value]
        top_cv_image = self.bridge.imgmsg_to_cv2(top_camera_frame, "bgr8")
        top_cv_image = cv2.cvtColor(top_cv_image, cv2.COLOR_RGB2RGBA)
        top_cv_image = self.job_type_image_edit.edit_top_camera_image(top_cv_image, metric_info)
        edited_top_frame = self.bridge.cv2_to_imgmsg(top_cv_image, "bgr8")
        edited_frame_result[FrameTypes.TOP_CAMERA_FRAME.value] = edited_top_frame

    def _edit_camera_images(self, frame_data):
        """ Edit camera image by calling job type editor

        Arguments:
            frame_data (dict): Dictionary of frame, agent_metric_info, training_phase

        Returns:
            Image: Edited image
        """
        metric_info = {
            FrameQueueData.AGENT_METRIC_INFO.value: frame_data[FrameQueueData.AGENT_METRIC_INFO.value],
            FrameQueueData.TRAINING_PHASE.value: frame_data[FrameQueueData.TRAINING_PHASE.value]
        }

        # convert ros image message to cv image
        try:
            edited_frame_result = dict()
            self._edit_main_camera_images(frame_data, metric_info, edited_frame_result)
            # Edit top camera image only if its F1
            edited_frame_result[FrameTypes.TOP_CAMERA_FRAME.value] = None
            if self.top_camera_mp4_pub:
                self._edit_top_camera_images(frame_data, metric_info, edited_frame_result)
            return edited_frame_result
        except CvBridgeError as ex:
            LOG.info("cv2 to ROS image message error: {}".format(ex))

    def get_latest_frame_metric_data(self):
        """ Get the latest frame, metric information, training phase data from the double buffer.

        Returns:
            queue_data (dict): Contains information of the frame, agent_metric_info, training_phase
        """
        agent_metric_info = []
        has_uninitialized_metric = False
        for idx, metrics in enumerate(self._agents_metrics):
            try:
                latest_metric = metrics.get(block=False)
                if latest_metric is not None:
                    self._latest_agent_metric_info[idx] = latest_metric
            except DoubleBuffer.Empty:
                pass

            cached_metric = self._latest_agent_metric_info[idx]
            if cached_metric is None:
                has_uninitialized_metric = True
            agent_metric_info.append(cached_metric)

        if has_uninitialized_metric:
            return None

        queue_data = {
            FrameQueueData.FRAME.value: {
                FrameTypes.MAIN_CAMERA_FRAME.value: self._main_camera_frame_buffer.get(),
                FrameTypes.TOP_CAMERA_FRAME.value: (self._top_camera_frame_buffer.get()
                                                    if self.top_camera_mp4_pub else [])
            },
            FrameQueueData.AGENT_METRIC_INFO.value: agent_metric_info,
            FrameQueueData.TRAINING_PHASE.value: self._current_training_phase.get() if self.is_training else ''
        }
        return queue_data

    def _producer_frame_thread(self, frame):
        """ Callback for the main camera frame. Once a new image is received, all the required
        service calls are made to get the video metric information. Then for Mp4 its put into the queue
        but for the KVS its put into a double since we only care for the latest image for KVS

        Arguments:
            frame (cv2.ImgMsg): Image/Sensor topic of the camera image frame
        """
        if rclpy.ok():
            self._main_camera_frame_buffer.put(frame)

            # Get frame from main camera & agents metric information
            frame_metric_data = self.get_latest_frame_metric_data()

            if frame_metric_data is None:
                return

            # Queue frames if MP4 saving OR KVS streaming is enabled
            if self.is_save_mp4_enabled or self._is_publish_to_kvs_stream:
                if self._agent_frame_queue.qsize() == MAX_FRAMES_IN_QUEUE:
                    LOG.info("Dropping frame from the queue")
                    self._agent_frame_queue.get()
                # Append to the queue for processing
                self._agent_frame_queue.put(frame_metric_data)

    def _consumer_mp4_frame_thread(self):
        """ Consumes the frame produced by the _producer_frame_thread and edits the image
        The edited image is put into another queue for publishing to MP4 topic
        """
        while rclpy.ok():
            frame_data = None
            try:
                # Pop from the queue and edit the image
                frame_data = self._agent_frame_queue.get(timeout=QUEUE_WAIT_TIME)
            except queue.Empty:
                LOG.debug("AgentsVideoEditor._frame_queue['{}'] is empty. Retrying...".format(self.racecar_index))
            if frame_data:
                edited_frames = self._edit_camera_images(frame_data)
                # Publish to MP4 topics only if MP4 saving is enabled
                if self.is_save_mp4_enabled or self._agent_frame_queue_to_complete > 0:
                    if self._agent_frame_queue_to_complete > 0:
                        self._agent_frame_queue_to_complete -= 1
                    self.mp4_main_camera_pub.publish(edited_frames[FrameTypes.MAIN_CAMERA_FRAME.value])
                    if self.top_camera_mp4_pub:
                        self.top_camera_mp4_pub.publish(edited_frames[FrameTypes.TOP_CAMERA_FRAME.value])
                # Update KVS buffer with edited frame (if KVS is enabled)
                if self._is_publish_to_kvs_stream:
                    self._kvs_edited_frame_buffer.put(edited_frames[FrameTypes.MAIN_CAMERA_FRAME.value])

    def _kvs_publisher(self):
        """ Publishing the latest edited image to KVS topic at 15 FPS real time.

        In case of Kinesis video stream we want to publish frames real time at 15 FPS. If the frames
        are not published at this rate, there will be jitter and video will be laggy. So it has to always
        be the real time. Unlike mp4_publisher this cannot be a simulation time.
        """
        try:
            prev_time = time.time()
            while rclpy.ok():
                if len(self.racecars_info) > len(self._agents_metrics):
                    # Waiting for all the agents to initialize before editing videos
                    # There could be condition when racecar_0 starts editing frames before racecar_1 is initialized
                    time.sleep(KVS_PUBLISH_PERIOD)
                    continue
                elif len(self.racecars_info) < len(self._agents_metrics):
                    log_and_exit("Agents video editing metric cannot be larger than racecar info",
                                 SIMAPP_SIMULATION_KINESIS_VIDEO_CAMERA_EXCEPTION,
                                 SIMAPP_EVENT_ERROR_CODE_500)
                # Get latest edited frame from buffer (already edited by MP4 consumer)
                edited_frame = self._kvs_edited_frame_buffer.get()
                if rclpy.ok() and edited_frame is not None:
                    self.kvs_pub.publish(edited_frame)
                    cur_time = time.time()
                    time_diff = cur_time - prev_time
                    time.sleep(max(KVS_PUBLISH_PERIOD - time_diff, 0))
                    prev_time = time.time()
        except:
            pass


def get_racecars_info(node, racecar_names):
    """ This function returns the agents information like name, car color, display name
    Arguments:
        node (Node): ROS 2 node for parameter access
        racecar_names (list): comma seperated racecar names
    Returns:
        (list): Racecar information such as name, car color, display name
    """
    racecars = racecar_names
    racecars_info = list()
    
    # Declare parameter with default value
    node.declare_parameter('CAR_COLOR', force_list(WorldConfig.get_param('CAR_COLOR', [DEFAULT_COLOR] * len(racecar_names))))
    racecars_color = force_list(node.get_parameter('CAR_COLOR').get_parameter_value().string_array_value)
    
    # If parameter is empty, use default
    if not racecars_color:
        racecars_color = [DEFAULT_COLOR] * len(racecar_names)
    
    racecars_display_name = get_video_display_name()

    for i, racecar_name in enumerate(racecars):
        racecar_dict = dict()
        racecar_dict['name'] = racecar_name
        racecar_dict['racecar_color'] = racecars_color[i]
        racecar_dict['display_name'] = racecars_display_name[i]
        racecars_info.append(racecar_dict)
    return racecars_info


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    executor = None
    temp_node = None
    video_editor_nodes = []
    
    try:
        # comma separated racecar names passed as an argument to the node
        RACER_NUM = int(sys.argv[1]) if len(sys.argv) > 1 else 1
        RACECAR_NAMES = get_racecar_names(RACER_NUM)
        PUBLISH_TO_KVS_STREAM = False if len(sys.argv) > 2 and sys.argv[2].lower() == "false" else True
        LOG.info(f"Publishing to KVS stream is enabled: {PUBLISH_TO_KVS_STREAM}")
        
        # Create a temporary node for parameter access
        temp_node = Node('temp_param_node')
        
        racecars_info = get_racecars_info(temp_node, RACECAR_NAMES)
        
        # Destroy temporary node
        temp_node.destroy_node()
        
        for racecar in racecars_info:
            # Instantiate AgentCameraVideoEditor objects for each racecar
            video_editor = AgentsVideoEditor(racecar['name'], racecars_info, PUBLISH_TO_KVS_STREAM)
            video_editor_nodes.append(video_editor)
        
        # Create executor to spin all video editor nodes
        executor = MultiThreadedExecutor()
        for video_editor in video_editor_nodes:
            executor.add_node(video_editor)
            executor.add_node(video_editor.save_to_mp4_obj)
        
        try:
            # Keep the nodes alive to handle service calls
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            LOG.info("agents_video_editor shutdown requested")
        except Exception as spin_error:
            LOG.error("Error in executor spin: %s", spin_error)
            raise
        
    except (KeyboardInterrupt, ExternalShutdownException):
        LOG.info("agents_video_editor shutdown requested")
    except Exception as err_msg:
        log_and_exit("Exception in Kinesis Video camera ros node: {}".format(err_msg),
                     SIMAPP_SIMULATION_KINESIS_VIDEO_CAMERA_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)
    finally:
        for video_editor in video_editor_nodes:
            try:
                video_editor.request_shutdown()
            except Exception:
                pass

        if executor is not None:
            try:
                executor.shutdown(timeout_sec=2.0)
            except Exception:
                pass

        if temp_node is not None:
            try:
                temp_node.destroy_node()
            except Exception:
                pass

        for video_editor in video_editor_nodes:
            try:
                if hasattr(video_editor, 'save_to_mp4_obj') and video_editor.save_to_mp4_obj is not None:
                    video_editor.save_to_mp4_obj.destroy_node()
            except Exception:
                pass
            try:
                video_editor.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
