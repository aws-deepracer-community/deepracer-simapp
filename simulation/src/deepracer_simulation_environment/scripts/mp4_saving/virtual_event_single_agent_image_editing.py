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

""" Image editing class for virtual event only
"""

import os
import datetime
import logging
import rospy
import cv2

from markov.log_handler.logger import Logger
from markov.utils import get_racecar_idx
from mp4_saving import utils
from mp4_saving.constants import (RaceCarColorToRGB,
                                  IconographicImageSize,
                                  SCALE_RATIO, FrameQueueData,
                                  VirtualEventMP4Params,
                                  VirtualEventIconographicPngs,
                                  VirtualEventXYPixelLoc, VirtualEventFader,
                                  VirtualEventData)
from mp4_saving.image_editing_interface import ImageEditingInterface
from mp4_saving.top_view_graphics import TopViewGraphics
from mp4_saving.fader import Fader
from markov.virtual_event.constants import DEFAULT_RACE_DURATION
from mp4_saving.states.virtual_event_wait_state import VirtualEventWaitState
from markov.state_machine.fsm import FSM
from markov.boto.s3.files.virtual_event_best_sector_time import VirtualEventBestSectorTime
from markov.boto.s3.constants import (SECTOR_TIME_LOCAL_PATH,
                                      SECTOR_TIME_S3_POSTFIX,
                                      TrackSectorTime,
                                      SECTOR_X_FORMAT,
                                      SECTOR_TIME_FORMAT_DICT)
from markov.boto.s3.utils import get_s3_key

LOG = Logger(__name__, logging.INFO).get_logger()


class VirtualEventSingleAgentImageEditing(ImageEditingInterface):
    """ Image editing class for virtual event
    """
    def __init__(self, racecar_name, racecar_info, race_type):
        """ Initializing the required data for the head to bot, time-trail. This is used for single agent
        Arguments:
            racecar_name (str): racecar name in string
            racecars_info (list): list of dict having information of the agent
            race_type (str): Since this class is reused for all the different race_type
        """
        # race duration in milliseconds
        self._world_name = rospy.get_param("WORLD_NAME")
        self.num_sectors = int(rospy.get_param("NUM_SECTORS", "3"))
        self.race_duration = int(rospy.get_param("RACE_DURATION", DEFAULT_RACE_DURATION)) * 1000
        self.racecar_info = racecar_info
        self.race_type = race_type
        racecar_index = get_racecar_idx(racecar_name)
        self.racecar_index = racecar_index if racecar_index else 0
        # Store the font which we will use to write the phase with
        self.amazon_ember_regular_28px = utils.get_font('AmazonEmber-Regular', 28)
        self.amazon_ember_regular_14px = utils.get_font('AmazonEmber-Regular', 14)

        # The track image as iconography
        self.track_icongraphy_img = utils.get_track_iconography_image()

        # Track image offset
        self.track_loc_offset = VirtualEventXYPixelLoc.TRACK_IMG_VIRTUAL_EVENT_LOC.value
        self._track_x_min = None
        self._track_x_max = None
        self._track_y_min = None
        self._track_y_max = None

        # Gradient overlay image with track and virtual event mock
        gradient_img_path = VirtualEventIconographicPngs.OVERLAY_PNG.value
        self.gradient_img = self._plot_track_on_gradient(gradient_img_path)

        # Time remaining text
        loc_x, loc_y = VirtualEventXYPixelLoc.TIME_REMAINING_TEXT.value
        self.gradient_img = utils.write_text_on_image(image=self.gradient_img, text="TIME REMAINING",
                                                      loc=(loc_x, loc_y), font=self.amazon_ember_regular_14px,
                                                      font_color=RaceCarColorToRGB.White.value,
                                                      font_shadow_color=RaceCarColorToRGB.Black.value)

        # Speed text
        loc_x, loc_y = VirtualEventXYPixelLoc.SPEED_TEXT.value
        self.gradient_img = utils.write_text_on_image(image=self.gradient_img, text="m/s",
                                                      loc=(loc_x, loc_y), font=self.amazon_ember_regular_14px,
                                                      font_color=RaceCarColorToRGB.White.value,
                                                      font_shadow_color=RaceCarColorToRGB.Black.value)

        # Reset text
        loc_x, loc_y = VirtualEventXYPixelLoc.RESET_TEXT.value
        self.gradient_img = utils.write_text_on_image(image=self.gradient_img, text="RESET",
                                                      loc=(loc_x, loc_y), font=self.amazon_ember_regular_14px,
                                                      font_color=RaceCarColorToRGB.White.value,
                                                      font_shadow_color=RaceCarColorToRGB.Black.value)

        # current lap time text
        loc_x, loc_y = VirtualEventXYPixelLoc.CURRENT_LAP_TIME_TEXT.value
        self.gradient_img = utils.write_text_on_image(image=self.gradient_img, text="CURRENT LAP TIME",
                                                      loc=(loc_x, loc_y), font=self.amazon_ember_regular_14px,
                                                      font_color=RaceCarColorToRGB.White.value,
                                                      font_shadow_color=RaceCarColorToRGB.Black.value)

        # best lap time text
        loc_x, loc_y = VirtualEventXYPixelLoc.BEST_LAP_TIME_TEXT.value
        self.gradient_img = utils.write_text_on_image(image=self.gradient_img, text="BEST LAP TIME",
                                                      loc=(loc_x, loc_y), font=self.amazon_ember_regular_14px,
                                                      font_color=RaceCarColorToRGB.White.value,
                                                      font_shadow_color=RaceCarColorToRGB.Black.value)

        # apply graident
        self.gradient_alpha_rgb_mul, self.one_minus_gradient_alpha = utils.get_gradient_values(self.gradient_img)

        # Top camera information
        top_camera_info = utils.get_top_camera_info()
        self.top_view_graphics = TopViewGraphics(top_camera_info.horizontal_fov, top_camera_info.padding_pct,
                                                 top_camera_info.image_width, top_camera_info.image_height,
                                                 racecar_info,
                                                 is_virtual_event=True)

        # virtual event image editting state machine
        self._image_edit_fsm = FSM(initial_state=VirtualEventWaitState())
        # if best sector time download from s3 failed. Then, initialize best sector time as None
        # and not display sector color
        self._sector_times = {}

        # declare sector images
        self._sectors_img_dict = {}
        for idx in range(self.num_sectors):
            sector = SECTOR_X_FORMAT.format(idx + 1)
            sector_color_img_dict = utils.init_sector_img_dict(world_name=self._world_name,
                                                               sector=sector)
            self._sectors_img_dict[sector] = sector_color_img_dict

        # use the s3 bucket and prefix for yaml file stored as environment variable because
        # here is SimApp use only. For virtual event there is no s3 bucket and prefix past
        # through yaml file. All are past through sqs. For simplicity, reuse the yaml s3 bucket
        # and prefix environment variable.
        self._virtual_event_best_sector_time = VirtualEventBestSectorTime(
            bucket=os.environ.get("YAML_S3_BUCKET", ''),
            s3_key=get_s3_key(os.environ.get("YAML_S3_PREFIX", ''), SECTOR_TIME_S3_POSTFIX),
            region_name=os.environ.get("APP_REGION", "us-east-1"),
            local_path=SECTOR_TIME_LOCAL_PATH)
        self._sector_times.update(self._virtual_event_best_sector_time.get_sector_time(
            num_sectors=self.num_sectors))

        # declare default best personal and current persoanl time to inf
        for idx in range(self.num_sectors):
            sector = SECTOR_X_FORMAT.format(idx + 1)
            self._sector_times[SECTOR_TIME_FORMAT_DICT[TrackSectorTime.BEST_PERSONAL].format(sector)] = float("inf")
            self._sector_times[SECTOR_TIME_FORMAT_DICT[TrackSectorTime.CURRENT_PERSONAL].format(sector)] = float("inf")

        self._curr_lap_time = 0
        self._last_eval_time = 0
        self._curr_progress = 0
        self._last_progress = 0

        # Initializing the fader behaviour to pre-compute the gradient values
        final_fading_image = utils.get_image(VirtualEventIconographicPngs.FINAL_FADING_IMAGE_50ALPHA.value,
                                             IconographicImageSize.FULL_IMAGE_SIZE.value)
        final_fading_image = cv2.cvtColor(final_fading_image, cv2.COLOR_RGBA2BGRA)
        self._fader_obj = Fader(final_fading_image, fading_min_percent=VirtualEventFader.FADING_MIN_PERCENT.value,
                                fading_max_percent=VirtualEventFader.FADING_MAX_PERCENT.value,
                                num_frames=VirtualEventFader.NUM_FRAMES.value)

    def _edit_major_cv_image(self, major_cv_image, metric_info):
        """ Apply all the editing for the Major 45degree camera image
        Args:
            major_cv_image (Image): Image straight from the camera
            metric_info (dict): rest image editting info
        Returns:
            Image: Edited main camera image
        """
        major_cv_image = utils.apply_gradient(major_cv_image, self.gradient_alpha_rgb_mul,
                                              self.one_minus_gradient_alpha)
        #########################
        # update display params #
        #########################
        mp4_video_metrics_info = metric_info[FrameQueueData.AGENT_METRIC_INFO.value]
        virtual_event_info = metric_info[FrameQueueData.VIRTUAL_EVENT_INFO.value]
        episode_status = mp4_video_metrics_info[self.racecar_index].episode_status
        # total_evaluation_time (Race time)
        total_eval_milli_seconds = mp4_video_metrics_info[self.racecar_index].total_evaluation_time
        # Reset counter
        reset_counter = mp4_video_metrics_info[self.racecar_index].reset_counter
        # Speed
        speed = mp4_video_metrics_info[self.racecar_index].throttle
        # Current progress
        current_progress = mp4_video_metrics_info[self.racecar_index].completion_percentage
        # Prepare a dict for finite state machine on event call
        info_dict = {VirtualEventMP4Params.COUNTDOWN_TIMER.value: mp4_video_metrics_info[self.racecar_index].pause_duration,
                     VirtualEventMP4Params.MAJOR_CV_IMAGE.value: major_cv_image,
                     VirtualEventMP4Params.CURRENT_LAP.value: virtual_event_info[VirtualEventData.LAP.value] + 1,
                     VirtualEventMP4Params.TOTAL_EVAL_SECONDS.value: total_eval_milli_seconds,
                     VirtualEventMP4Params.RESET_COUNTER.value: reset_counter,
                     VirtualEventMP4Params.SPEED.value: speed,
                     VirtualEventMP4Params.CURR_PROGRESS.value: current_progress,
                     VirtualEventMP4Params.LAST_EVAL_SECONDS.value: self._last_eval_time,
                     VirtualEventMP4Params.X_MIN.value: self._track_x_min,
                     VirtualEventMP4Params.X_MAX.value: self._track_x_max,
                     VirtualEventMP4Params.Y_MIN.value: self._track_y_min,
                     VirtualEventMP4Params.Y_MAX.value: self._track_y_max,
                     VirtualEventMP4Params.SECTOR_TIMES.value: self._sector_times,
                     VirtualEventMP4Params.CURR_LAP_TIME.value: self._curr_lap_time,
                     VirtualEventMP4Params.SECTOR_IMAGES.value: self._sectors_img_dict,
                     VirtualEventMP4Params.FADER_OBJ.value: self._fader_obj}

        #####################
        # run state machine #
        #####################
        # virtual event image edit finite state machine on event
        info_dict = self._image_edit_fsm.execute(input_val={'event': episode_status,
                                                            'info_dict': info_dict})

        # update display param from the finite state machine return value
        major_cv_image = info_dict[VirtualEventMP4Params.MAJOR_CV_IMAGE.value]
        total_eval_milli_seconds = info_dict[VirtualEventMP4Params.TOTAL_EVAL_SECONDS.value]
        reset_counter = info_dict[VirtualEventMP4Params.RESET_COUNTER.value]
        speed = info_dict[VirtualEventMP4Params.SPEED.value]
        self._last_eval_time = info_dict[VirtualEventMP4Params.LAST_EVAL_SECONDS.value]
        self._sector_times = info_dict[VirtualEventMP4Params.SECTOR_TIMES.value]
        self._curr_lap_time = info_dict[VirtualEventMP4Params.CURR_LAP_TIME.value]

        # Time remaining digit
        loc_x, loc_y = VirtualEventXYPixelLoc.TIME_REMAINING_DIGIT.value
        time_remaining = self.race_duration - total_eval_milli_seconds
        time_remaining = time_remaining if time_remaining > 0.0 else 0.0
        time_remaining = datetime.timedelta(milliseconds=time_remaining)
        time_remaining = utils.milliseconds_to_timeformat(time_remaining)
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=time_remaining,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_regular_28px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)

        # Speed digit
        loc_x, loc_y = VirtualEventXYPixelLoc.SPEED_DIGIT.value
        speed_text = utils.get_speed_formatted_str(speed)
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=speed_text,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_regular_28px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)

        # Reset digit
        loc_x, loc_y = VirtualEventXYPixelLoc.RESET_DIGIT.value
        reset_counter_text = "{}".format(reset_counter)
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=reset_counter_text,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_regular_28px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)

        # curent lap time digit
        loc_x, loc_y = VirtualEventXYPixelLoc.CURRENT_LAP_TIME_DIGIT.value
        curr_lap_time = utils.milliseconds_to_timeformat(
            datetime.timedelta(milliseconds=self._curr_lap_time))
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=curr_lap_time,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_regular_28px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)

        # best lap time digit
        loc_x, loc_y = VirtualEventXYPixelLoc.BEST_LAP_TIME_DIGIT.value
        best_lap_time = mp4_video_metrics_info[self.racecar_index].best_lap_time
        # The initial default best_lap_time from s3_metrics.py is inf
        # If the ros service in s3_metrics.py has not come up yet, best_lap_time is 0
        best_lap_time = utils.milliseconds_to_timeformat(
            datetime.timedelta(milliseconds=best_lap_time)) \
            if best_lap_time != float("inf") and best_lap_time != 0 else "--:--.---"
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=best_lap_time,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_regular_28px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)

        major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2BGRA)
        return major_cv_image

    def _plot_track_on_gradient(self, gradient_img_path):
        """ For the given gradient apply the track iconographic image and use this to apply gradient
        on each camera frame. Previously this was done on the top camera which changed every frame. But
        with the track iconographic image set static, adding the track on gradient is more optimized.

        Arguments:
            gradient_img_path (str): Gradient image path

        Returns:
            (Image): Edited gradient image with track image
        """
        gradient_img = utils.get_image(gradient_img_path, IconographicImageSize.FULL_IMAGE_SIZE.value)
        gradient_img = cv2.cvtColor(gradient_img, cv2.COLOR_RGBA2BGRA)

        track_icongraphy_scaled = utils.resize_image(self.track_icongraphy_img, SCALE_RATIO)
        track_icongraphy_alpha = track_icongraphy_scaled[:, :, 3] / 255.0

        # Track image is placed at the top left
        self._track_x_min = self.track_loc_offset[1]
        self._track_x_max = self.track_loc_offset[1] + track_icongraphy_scaled.shape[0]
        self._track_y_min = self.track_loc_offset[0]
        self._track_y_max = self.track_loc_offset[0] + track_icongraphy_scaled.shape[1]

        # This is used as the offset for plotting the agent dots
        self.track_start_loc = (self._track_y_min, self._track_x_min)

        for channel in range(0, 4):
            gradient_img[self._track_x_min:self._track_x_max, self._track_y_min:self._track_y_max, channel] =\
                (track_icongraphy_alpha * track_icongraphy_scaled[:, :, channel]) + \
                (1 - track_icongraphy_alpha) * (gradient_img[self._track_x_min:self._track_x_max, self._track_y_min:self._track_y_max, channel])
        return gradient_img

    def _plot_agents_on_major_cv_image(self, major_cv_image, mp4_video_metrics_info):
        """ Add the agents, obstacles on the track.

        Arguments:
            major_cv_image (Image): Edited image having gradient, text, track
            mp4_video_metrics_info (List): List of ROS metric values of each agent

        Returns:
            Image: Edited image with gradient, text, track and agents with dots
        """
        agents_loc = [(metric.x, metric.y) for metric in mp4_video_metrics_info]
        objects_loc = []
        if mp4_video_metrics_info[0].object_locations:
            objects_loc = [(object_loc.x, object_loc.y) for object_loc in mp4_video_metrics_info[0].object_locations]
        return self.top_view_graphics.plot_agents_as_circles(
            major_cv_image, agents_loc, objects_loc, self.track_start_loc)

    def edit_image(self, major_cv_image, metric_info):
        major_cv_image = self._edit_major_cv_image(
            major_cv_image,
            metric_info)
        major_cv_image = self._plot_agents_on_major_cv_image(
            major_cv_image,
            metric_info[FrameQueueData.AGENT_METRIC_INFO.value])
        return cv2.cvtColor(major_cv_image, cv2.COLOR_BGRA2RGB)
