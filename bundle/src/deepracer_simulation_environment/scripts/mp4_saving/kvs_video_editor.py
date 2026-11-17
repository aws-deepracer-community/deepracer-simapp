#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

""" Dedicated node that renders the overlay for the live Kinesis Video (KVS) stream.

This is split out of agents_video_editor so the CPU-heavy overlay editing runs in its
OWN process. When the editing ran in the same process as the camera-frame intake
(as a separate publisher thread), it held the Python GIL and starved the intake
callback, so the "latest frame" went stale and the live evaluation stream dropped
~40% of unique frames (visible choppiness). MP4 recording and training are unaffected:
agents_video_editor still owns the MP4 path and the camera intake; this node only
produces the KVS stream.

The overlay is edited INSIDE the camera subscription callback (not a separate thread),
so within this dedicated process there is no editing-thread-vs-intake contention - each
camera frame is edited and published as it arrives. A MultiThreadedExecutor is used so
the per-frame metrics service call can be serviced while a callback is in flight (same
pattern agents_video_editor relies on).
"""
import sys
import time
import logging
import threading
import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ROSImg
from std_msgs.msg import String

from markov.utils import DoubleBuffer, get_racecar_names, get_racecar_idx
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_KINESIS_VIDEO_CAMERA_EXCEPTION)
from markov.reset.constants import RaceType
from markov.world_config import WorldConfig
from deepracer_simulation_environment.msg import VideoMetrics
from mp4_saving.constants import FrameQueueData, FrameTypes, KVS_PUBLISH_PERIOD
from mp4_saving.single_agent_image_editing import SingleAgentImageEditing
from mp4_saving.multi_agent_image_editing import MultiAgentImageEditing
from mp4_saving.training_image_editing import TrainingImageEditing
from mp4_saving.f1_image_editing import F1ImageEditing
from mp4_saving import utils
from mp4_saving.agents_video_editor import get_racecars_info

LOG = Logger(__name__, logging.INFO).get_logger()

# Only the freshest frame matters for the live stream, so keep a depth of 1.
CAMERA_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class KvsVideoEditor(Node):
    """ Edits and publishes the live KVS stream for a single racecar, in its own process. """

    def __init__(self, racecar_name, racecars_info):
        super().__init__('kvs_video_editor_{}'.format(racecar_name))
        self.bridge = CvBridge()
        self.racecar_name = racecar_name
        self.racecars_info = racecars_info
        racecar_index = get_racecar_idx(racecar_name)
        self.racecar_index = racecar_index if racecar_index else 0
        self.agent_name = utils.racecar_name_to_agent_name(racecars_info, racecar_name)

        self.declare_parameter('JOB_TYPE', WorldConfig.get_param('JOB_TYPE', 'EVALUATION'))
        self.declare_parameter('RACE_TYPE', WorldConfig.get_param('RACE_TYPE', RaceType.TIME_TRIAL.value))
        self.is_training = self.get_parameter('JOB_TYPE').get_parameter_value().string_value == 'TRAINING'
        self.race_type = self.get_parameter('RACE_TYPE').get_parameter_value().string_value
        self.is_f1_race_type = self.race_type == RaceType.F1.value

        self.job_type_image_edit_kvs = self._get_image_editing_job_type()

        # Training phase label (only present for training videos)
        self._current_training_phase = None
        if self.is_training:
            self._current_training_phase = DoubleBuffer(clear_data_on_get=False)
            self._current_training_phase.put('Initializing')
            self.create_subscription(String, '/agent/training_phase',
                                     self._training_phase_cb, QoSProfile(depth=10))

        # One metrics buffer per agent populated via the /{agent}/video_metrics topic.
        # Non-blocking reads mean the camera callback is never delayed waiting for a response.
        self._metrics_buffers = {}
        self._metrics_subs = []
        for racecar in racecars_info:
            agent_name = utils.racecar_name_to_agent_name(racecars_info, racecar['name'])
            buf = DoubleBuffer(clear_data_on_get=False)
            self._metrics_buffers[agent_name] = buf
            # Keep a reference so the subscription is not garbage-collected.
            self._metrics_subs.append(
                self.create_subscription(
                    VideoMetrics,
                    f'/{agent_name}/video_metrics',
                    lambda msg, a=agent_name: self._metrics_buffers[a].put(msg),
                    10
                )
            )

        self.kvs_pub = self.create_publisher(
            ROSImg, '/{}/deepracer/kvs_stream'.format(self.racecar_name), 1)

        # Frame counters for the end-of-run summary (plain ints; approximate across threads)
        self._camera_rx = 0
        self._skipped_no_metrics = 0
        self._edited = 0
        self._published = 0
        self._errors = 0

        # Latest edited frame, published on a steady cadence (below). Editing happens in
        # the camera callback; publishing is decoupled so the live stream keeps a steady
        # ~15 FPS cadence (re-sending the latest frame when no newer one is ready). A
        # variable, camera-rate cadence makes the console player rebuffer (spinner).
        self._latest_edited = DoubleBuffer(clear_data_on_get=False)

        main_camera_topic = "/{}/{}/zed/rgb/image_rect_color".format(self.racecar_name, "main_camera")
        # Edit inside the camera callback (no separate editing thread to contend with the
        # intake for the GIL); store the result for the steady-rate publisher. Put the
        # subscription in its own callback group (mirrors agents_video_editor) so the heavy
        # edit callback never blocks other callbacks the executor needs to service.
        self._camera_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(ROSImg, main_camera_topic, self._on_camera_frame, CAMERA_QOS,
                                 callback_group=self._camera_cb_group)

        # Steady-cadence publisher so the player does not rebuffer.
        self._stop_event = threading.Event()
        self._publish_thread = threading.Thread(target=self._kvs_publish_loop, daemon=True)
        self._publish_thread.start()

    def stop(self):
        """ Stop the publish thread so nothing touches the publisher while ROS is torn down. """
        self._stop_event.set()
        self._publish_thread.join(timeout=1.0)

    def _training_phase_cb(self, phase):
        self._current_training_phase.put(phase.data)

    def log_summary(self):
        """ End-of-run summary of how many camera frames were edited and published. """
        LOG.info("kvs_video_editor[%s] summary: camera rx=%d skipped(no metrics)=%d edited=%d "
                 "published=%d errors=%d",
                 self.racecar_name, self._camera_rx, self._skipped_no_metrics,
                 self._edited, self._published, self._errors)

    def _get_image_editing_job_type(self):
        """ Same editor selection as agents_video_editor so output is identical. """
        if self.is_training:
            return TrainingImageEditing(self.racecar_name, self.racecars_info, self.race_type)
        if self.is_f1_race_type:
            return F1ImageEditing(self.racecar_name, self.racecars_info, self.race_type)
        if self.race_type == RaceType.HEAD_TO_MODEL.value:
            return MultiAgentImageEditing(self.racecar_name, self.racecars_info, self.race_type)
        if self.race_type in [RaceType.TIME_TRIAL.value, RaceType.OBJECT_AVOIDANCE.value,
                              RaceType.HEAD_TO_BOT.value]:
            return SingleAgentImageEditing(self.racecar_name, self.racecars_info, self.race_type)
        raise Exception("Unknown job type for image editing")

    def _on_camera_frame(self, frame):
        """ Edit the freshest camera frame with the overlay and store it for publishing. """
        if not rclpy.ok():
            return
        self._camera_rx += 1
        try:
            # Collect metrics from each agent's buffer (non-blocking).
            # If any buffer has not yet received data, skip this frame to avoid a crash.
            agent_metric_info = []
            for racecar in self.racecars_info:
                agent_name = utils.racecar_name_to_agent_name(self.racecars_info, racecar['name'])
                buf = self._metrics_buffers.get(agent_name)
                try:
                    metric = buf.get(block=False) if buf is not None else None
                except DoubleBuffer.Empty:
                    metric = None
                if metric is None:
                    self._skipped_no_metrics += 1
                    return  # metrics not yet available; skip frame
                agent_metric_info.append(metric)
            metric_info = {
                FrameQueueData.AGENT_METRIC_INFO.value: agent_metric_info,
                FrameQueueData.TRAINING_PHASE.value:
                    self._current_training_phase.get() if self.is_training else ''
            }
            major_cv_image = self.bridge.imgmsg_to_cv2(frame, "bgr8")
            major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2RGBA)
            major_cv_image = self.job_type_image_edit_kvs.edit_image(major_cv_image, metric_info)
            self._latest_edited.put(self.bridge.cv2_to_imgmsg(major_cv_image, "bgr8"))
            self._edited += 1
        except CvBridgeError as ex:
            self._errors += 1
            LOG.info("cv2 to ROS image message error: {}".format(ex))
        except Exception:
            self._errors += 1
            LOG.exception("kvs_video_editor[%s]: frame editing failed", self.racecar_name)
            raise

    def _kvs_publish_loop(self):
        """ Publish the latest edited frame at a steady ~15 FPS so the stream cadence is
        regular (no player rebuffering), re-sending the latest frame if no newer one is
        ready. Editing is done in the camera callback, so this loop stays light. """
        next_tick = time.monotonic()
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                edited_frame = self._latest_edited.get()  # blocks until first frame, then latest
                if rclpy.ok() and not self._stop_event.is_set() and edited_frame is not None:
                    self.kvs_pub.publish(edited_frame)
                    self._published += 1
            except Exception as ex:
                LOG.info("KVS publish loop error: {}".format(ex))
            finally:
                # Schedule on an absolute clock so the publish rate doesn't drift by the
                # time spent editing/publishing each iteration.
                next_tick += KVS_PUBLISH_PERIOD
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0:
                    self._stop_event.wait(sleep_time)


def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    editors = []
    executor = None
    try:
        racer_num = int(sys.argv[1]) if len(sys.argv) > 1 else 1
        racecar_names = get_racecar_names(racer_num)

        temp_node = Node('kvs_video_editor_param_node')
        racecars_info = get_racecars_info(temp_node, racecar_names)
        temp_node.destroy_node()

        executor = MultiThreadedExecutor()
        # This dedicated node is only for evaluation/race. In training the in-process
        # publisher in agents_video_editor handles KVS (its light overlay never starved
        # intake), so we stay alive but idle here to avoid adding load to training.
        if WorldConfig.get_param('JOB_TYPE', 'EVALUATION') == 'TRAINING':
            LOG.info("kvs_video_editor: training job - KVS handled by agents_video_editor; idling.")
            executor.add_node(Node('kvs_video_editor_idle'))
        else:
            for racecar in racecars_info:
                editor = KvsVideoEditor(racecar['name'], racecars_info)
                editors.append(editor)
                executor.add_node(editor)
            LOG.info("KVS video editor started for %d racecar(s)", len(racecars_info))
        executor.spin()
    except KeyboardInterrupt:
        pass  # Normal SIGINT shutdown — not an error
    except Exception as err_msg:
        log_and_exit("Exception in KVS video editor ros node: {}".format(err_msg),
                     SIMAPP_SIMULATION_KINESIS_VIDEO_CAMERA_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)
    finally:
        # Tear down in dependency order: stop the publish threads, stop the executor's worker
        # threads, then destroy the nodes, and only then shut rclpy down. Shutting the context
        # down first leaves threads using a dead publisher, which segfaults at interpreter exit.
        for editor in editors:
            editor.stop()
            editor.log_summary()
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=2.0)
            except Exception:
                pass
        for editor in editors:
            try:
                editor.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
