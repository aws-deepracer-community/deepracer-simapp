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

"""Image editing class for head to head where there are multiple agents
"""
import datetime
from collections import OrderedDict
import threading
import logging
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image as ROSImg

from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_SAVE_TO_MP4_EXCEPTION)
from markov.reset.constants import RaceType
from markov.utils import get_racecar_idx
from mp4_saving.top_view_graphics import TopViewGraphics
from mp4_saving.constants import (RaceCarColorToRGB,
                                  RACE_TYPE_TO_VIDEO_TEXT_MAPPING, SCALE_RATIO,
                                  TrackAssetsIconographicPngs, IconographicImageSize,
                                  XYPixelLoc, RACE_COMPLETE_Y_OFFSET,
                                  FrameQueueData)
from mp4_saving import utils
from mp4_saving.image_editing_interface import ImageEditingInterface

LOG = Logger(__name__, logging.INFO).get_logger()


class F1ImageEditing(ImageEditingInterface):
    """Image editing class for F1 grand prix
    """
    _lock = threading.Lock()
    _finished_lap_time = OrderedDict()
    _leader_percentage_completion = list()
    _leader_elapsed_time = list()

    def __init__(self, racecar_name, racecars_info, race_type):
        """ This class is used for head to head racing where there are more than one agent
        Args:
            racecar_name (str): The agent name with 45degree camera view
            racecars_info (dict): All the agents information
            race_type (str): The type of race. This is used to know if its race type or evaluation
        """
        self.racecar_name = racecar_name
        self.racecars_info = racecars_info
        racecar_index = get_racecar_idx(racecar_name)
        self.racecar_index = racecar_index if racecar_index else 0
        self.race_type = race_type

        # Store the font which we will use to write the phase with
        self.formula1_display_regular_12px = utils.get_font('Formula1-Display-Regular', 12)
        self.formula1_display_regular_14px = utils.get_font('Formula1-Display-Regular', 14)
        self.formula1_display_regular_16px = utils.get_font('Formula1-Display-Regular', 16)
        self.formula1_display_wide_12px = utils.get_font('Formula1-Display-Wide', 12)
        self.formula1_display_bold_16px = utils.get_font('Formula1-Display-Bold', 16)

        self.total_laps = int(rospy.get_param("NUMBER_OF_TRIALS", 0))
        self.is_league_leaderboard = rospy.get_param("LEADERBOARD_TYPE", "") == "LEAGUE"
        self.leaderboard_name = rospy.get_param("LEADERBOARD_NAME", "")

        # The track image as iconography
        self.track_icongraphy_img = utils.get_track_iconography_image()

        # Track image offset
        self.track_loc_offset = XYPixelLoc.TRACK_IMG_WITHOUT_OFFSET_LOC.value

        # Default image of top view
        gradient_default_img_path = TrackAssetsIconographicPngs.F1_OVERLAY_DEFAULT_PNG.value
        self.gradient_default_img = self._plot_track_on_gradient(gradient_default_img_path)
        self.gradient_default_alpha_rgb_mul, self.one_minus_gradient_default_alpha = utils.get_gradient_values(
            self.gradient_default_img)

        # Midway track gradient
        gradient_midway_img_path = TrackAssetsIconographicPngs.F1_OVERLAY_MIDWAY_PNG.value
        self.gradient_midway_img = self._plot_track_on_gradient(gradient_midway_img_path)
        self.gradient_midway_alpha_rgb_mul, self.one_minus_gradient_midway_alpha = utils.get_gradient_values(
            self.gradient_midway_img)

        # Finisher track gradient
        gradient_finisher_img_path = TrackAssetsIconographicPngs.F1_OVERLAY_FINISHERS_PNG.value
        self.gradient_finisher_img = self._plot_track_on_gradient(gradient_finisher_img_path)
        self.gradient_finisher_alpha_rgb_mul, self.one_minus_gradient_finisher_alpha = utils.get_gradient_values(
            self.gradient_finisher_img)

        # Top camera gradient
        num_racers = len(self.racecars_info)
        if num_racers <= 8:
            # TODO: Add one box image and use 1 box image if number of racers are <= 4.
            gradient_top_camera_img_path = TrackAssetsIconographicPngs.F1_OVERLAY_TOPVIEW_2BOX_PNG.value
        elif num_racers <= 12:
            gradient_top_camera_img_path = TrackAssetsIconographicPngs.F1_OVERLAY_TOPVIEW_3BOX_PNG.value
        else:
            raise Exception("More than 12 racers are not supported for Grand Prix")

        gradient_top_camera_img = utils.get_image(gradient_top_camera_img_path,
                                                  IconographicImageSize.FULL_IMAGE_SIZE.value)
        gradient_top_camera_img = cv2.cvtColor(gradient_top_camera_img, cv2.COLOR_RGBA2BGRA)
        self.gradient_top_camera_alpha_rgb_mul, self.one_minus_gradient_top_camera_alpha = utils.get_gradient_values(
            gradient_top_camera_img)

        # Top camera information
        top_camera_info = utils.get_top_camera_info()
        self.edited_topview_pub = rospy.Publisher('/deepracer/topview_stream', ROSImg, queue_size=1)
        self.top_view_graphics = TopViewGraphics(top_camera_info.horizontal_fov, top_camera_info.padding_pct,
                                                 top_camera_info.image_width, top_camera_info.image_height,
                                                 racecars_info, race_type)
        self.hex_car_colors = [val['racecar_color'].split('_')[-1] for val in racecars_info]
        self._racer_color_code_rect_img = list()
        self._racer_color_code_slash_img = list()
        for car_color in self.hex_car_colors:
            # Rectangular png of racers
            racer_color_code_rect = "{}_{}".format(TrackAssetsIconographicPngs.F1_AGENTS_RECT_DISPLAY_ICON_PNG.value,
                                                   car_color)
            self._racer_color_code_rect_img.append(
                utils.get_image(racer_color_code_rect, IconographicImageSize.F1_RACER_RECT_DISPLAY_ICON_SIZE.value))
            # Slash png of racers
            racer_color_code_slash = "{}_{}".format(TrackAssetsIconographicPngs.F1_AGENTS_SLASH_DISPLAY_ICON_PNG.value,
                                                    car_color)
            racer_color_code_slash_img = utils.get_image(racer_color_code_slash,
                                                         IconographicImageSize.F1_RACER_SLASH_DISPLAY_ICON_SIZE.value)
            self._racer_color_code_slash_img.append(cv2.cvtColor(racer_color_code_slash_img, cv2.COLOR_RGBA2BGRA))

    def _get_racecar_ranking(self, racers_ranking, racer_number):
        """ Returns the rank of the racer given the racing number.
        This function considers the cars finished and uses the elapsed time for giving the ranking.

        Arguments:
            racers_ranking (OrderedDict): Sorted ordered dict with racer_number and progress percentage along with lap
            racer_number (int): This is the racers number assigned when kinesis video node is spwan
        Returns:
            (int): The rank of the racer
        """
        if racer_number in racers_ranking:
            rank = list(racers_ranking.keys()).index(racer_number) + 1
            # The difference between self.racecars_info and racers_ranking will give
            # the number of racers completed the race at given time frame.
            # Relying on self._finished_lap_time for the ranking is a bad idea as given racers_ranking may not
            # be from current time (can be from queued frame which is from the past),
            # where self._finished_lap_time always contains the latest information.
            rank += len(self.racecars_info) - len(racers_ranking)
            return rank
        else:
            # If racer_number is not in racers_ranking then given racer_number racer at the given time frame
            # has already completed the race, then it must rely on self._finished_lap_time to
            # retrieve its ranking as racers_ranking won't contain the given racer_number and
            # self._finished_lap_time is the ONLY source to retrieve its ranking.
            return list(self._finished_lap_time.keys()).index(racer_number) + 1

    def _get_gap_time(self, racers_ranking, racer_number, racer_metrics_info):
        """ Checks if a car has complete the race and considers the difference in the
        elapsed time as gap time. If no racers have completed the lap then this function
        considers _leader_percentage_completion which tracks all the data points of the
        leaders progress to the elapsed time.

        Arguments:
            racers_ranking (OrderedDict): Sorted ordered dict with racer_number and progress percentage along with lap
            racer_number (int): This is the racers number assigned when kinesis video node is spwan
            racer_metrics_info (dict): Given racers metric information
        """
        if len(racers_ranking) < len(self.racecars_info):
            # if size of racers_ranking is smaller than size of self.racecars_info, then
            # it means there is at least one racer completed the race.
            leader_elapsed_time = list(self._finished_lap_time.items())[0][1]
            if racer_number not in racers_ranking:
                # If given racer has been also completed the race then the gap time is
                # just difference between the race finish times of given racer and leader.
                # Otherwise, the gap time needs to be retrieved from interpolation estimation.
                return (self._finished_lap_time[racer_number] - leader_elapsed_time) / 1000
            # Once leader completed the race, it's unnecessary to update
            # self._leader_elapsed_time and self._leader_percentage_completion lists.
            # Thus, set race_leader_index to None to avoid updating these static lists.
            race_leader_index = None
        else:
            # If none of the racers completed the race then every progress and time of the leader
            # needs to be recorded.
            race_leader_index = list(racers_ranking.items())[0][0]

        with self._lock:
            try:
                if race_leader_index is not None:
                    if len(self._leader_elapsed_time) == 0 or \
                            (racer_metrics_info.total_evaluation_time > self._leader_elapsed_time[-1] and
                             racers_ranking[race_leader_index] > self._leader_percentage_completion[-1]):
                        # ONLY append leader's datapoint when there is an actual latest new datapoint.
                        self._leader_percentage_completion.append(racers_ranking[race_leader_index])
                        # Since the total evaluation time is same for all the racers
                        self._leader_elapsed_time.append(racer_metrics_info.total_evaluation_time)

                if racers_ranking[racer_number] < 0.0:
                    # If the progress of the racer is less than 0 then it means racer hasn't passed the start line yet.
                    # In such case, the gap time based on leader's elapsed time is meaningless.
                    # Thus, just return 0.
                    return 0
                leader_elapsed_time = np.interp(racers_ranking[racer_number], self._leader_percentage_completion,
                                                self._leader_elapsed_time)
                return (racer_metrics_info.total_evaluation_time - leader_elapsed_time) / 1000
            except Exception as ex:
                LOG.info("Failed to find race leaders elapsed time: {}".format(ex))
        return 0

    def _racers_rank_name_gap_time(self, racers_ranking, mp4_video_metrics_info):
        """ Gets all the racers information rank, name, gap time. This is used for
        showing duirng the halfway mark and finished lap. This is used for the
        top view camera editing.

        Arguments:
            racers_ranking (OrderedDict): Sorted ordered dict with racer_number and progress percentage along with lap
            mp4_video_metrics_info (list): All the racers metric information
        Returns:
            (list): Sorted list based on the ranking along with rank, name, gap time, racer_number
        """
        rank_name_gap_time = list()
        for i in range(len(self.racecars_info)):
            racer_number = int(self.racecars_info[i]['name'].split("_")[1])
            rank_name_gap_time.append([
                self._get_racecar_ranking(racers_ranking, racer_number),
                self.racecars_info[i]['display_name'],
                self._get_gap_time(racers_ranking, racer_number, mp4_video_metrics_info[racer_number]),
                racer_number
            ])
        return sorted(rank_name_gap_time, key=lambda item: item[0])

    def _default_edit(self, major_cv_image):
        """ This is used as a default edit.

        Arguments:
            major_cv_image (Image): Main camera image for the racecar

        Returns:
            major_cv_image (Image): Edited Main camera image
        """
        # Applying gradient to whole major image and then writing text
        # F1 logo at the top left
        # loc_x, loc_y = XYPixelLoc.F1_LOGO_LOC.value
        # f1_logo_image = utils.get_image(TrackAssetsIconographicPngs.F1_LOGO_PNG.value,
        #                                 IconographicImageSize.F1_LOGO_IMAGE_SIZE.value)
        # major_cv_image = utils.plot_rectangular_image_on_main_image(major_cv_image, f1_logo_image, (loc_x, loc_y))
        major_cv_image = utils.apply_gradient(major_cv_image, self.gradient_default_alpha_rgb_mul,
                                              self.one_minus_gradient_default_alpha)
        return major_cv_image

    def _basic_racer_info_display(self, major_cv_image, racers_ranking, cur_racer_metrics_info, show_racer_name=True):
        """ Basic display editting of the main camera following the car.
        All the information at the bottom left of the MP4 is edited here.

        Arguments:
            major_cv_image (Image): Main camera image for the racecar
            racers_ranking (OrderedDict): Sorted ordered dict with racer_number and progress percentage along with lap
            cur_racer_metrics_info (dict): Given racers metric information

        Keyword Arguments:
            show_racer_name (bool): All the other states call this. But the finisher state does not have enough
            space to show the display name.(default: {True})

        Returns:
            major_cv_image (Image): Edited Main camera image
        """
        if show_racer_name:
            # Adding display name to the image
            loc_x, loc_y = XYPixelLoc.F1_DISPLAY_NAME_LOC.value
            display_name = self.racecars_info[self.racecar_index]['display_name']
            display_name_txt = display_name if len(display_name) <= 6 else "{}".format(display_name[:6])
            major_cv_image = utils.write_text_on_image(image=major_cv_image, text=display_name_txt,
                                                       loc=(loc_x, loc_y), font=self.formula1_display_regular_14px,
                                                       font_color=RaceCarColorToRGB.White.value,
                                                       font_shadow_color=RaceCarColorToRGB.Black.value)
        # Racers ranking
        loc_x, loc_y = XYPixelLoc.F1_RANKING_LOC.value
        cur_rank = self._get_racecar_ranking(racers_ranking, self.racecar_index)
        rank_txt = "Rank {}/{}".format(cur_rank, len(self.racecars_info))
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=rank_txt,
                                                   loc=(loc_x, loc_y), font=self.formula1_display_regular_16px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)

        # Lap Counter|Total eval|gap time
        # Lap counter calculation
        loc_x, loc_y = XYPixelLoc.F1_LAP_EVAL_GAP_LOC.value
        current_lap = min(int(cur_racer_metrics_info.lap_counter) + 1, int(self.total_laps))
        lap_counter_text = "Lap {:2d} / {:2d}".format(current_lap, int(self.total_laps))
        # Total eval time
        if self.racecar_index in self._finished_lap_time:
            # If the racer finished the race then lock the total evaluation time to finished lap time.
            total_eval_milli_seconds = self._finished_lap_time[self.racecar_index]
        else:
            total_eval_milli_seconds = cur_racer_metrics_info.total_evaluation_time
        time_delta = datetime.timedelta(milliseconds=total_eval_milli_seconds)
        total_eval_time_text = "{}".format(utils.milliseconds_to_timeformat(time_delta))
        # Writing to the frame (Lap Counter|Total eval|gap time)
        lap_elapsed_gap_time = "{} | {} | Gap ".format(lap_counter_text, total_eval_time_text)
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=lap_elapsed_gap_time,
                                                   loc=(loc_x, loc_y), font=self.formula1_display_regular_16px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # Gap (In orange color)
        loc_x, loc_y = XYPixelLoc.F1_LAP_EVAL_GAP_VAL_LOC.value
        # For the racer ranking 0, we see small floating number for the gap. Hence making it zero
        if cur_rank == 1:
            gap_time = 0
        gap_time = self._get_gap_time(racers_ranking, self.racecar_index, cur_racer_metrics_info)
        gap_time_text = "+{:.3f}".format(gap_time)
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=gap_time_text,
                                                   loc=(loc_x, loc_y), font=self.formula1_display_regular_16px,
                                                   font_color=RaceCarColorToRGB.Orange.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)

        # Leaderboard name
        f1_water_mark_text = "{}".format(self.leaderboard_name)
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=f1_water_mark_text,
                                                   loc=XYPixelLoc.F1_LEADERBOARD_NAME_LOC.value,
                                                   font=self.formula1_display_regular_12px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)

        # Do all plotting of images at the end
        if show_racer_name:
            # Draw racer color code
            loc_x, loc_y = XYPixelLoc.F1_DISPLAY_NAME_SLASH_LOC.value
            major_cv_image = utils.plot_rectangular_image_on_main_image(
                major_cv_image, self._racer_color_code_slash_img[self.racecar_index], (loc_x, loc_y))
        major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2BGRA)
        return major_cv_image

    def _midway_racers_progress_display(self, major_cv_image, rank_name_gap_time, mp4_video_metrics_info):
        """ Half way of the track we show the stats of the racers with there ranks and gap time

        Arguments:
            major_cv_image (Image): Main camera image for the racecar
            rank_name_gap_time (list): Sorted list based on the ranking along with rank, name, gap time, racer_number
            mp4_video_metrics_info (list): All the racers metric information

        Returns:
            major_cv_image (Image): Edited Main camera image
        """
        # Applying gradient to whole major image and then writing text
        major_cv_image = utils.apply_gradient(major_cv_image, self.gradient_midway_alpha_rgb_mul,
                                              self.one_minus_gradient_midway_alpha)
        major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_BGR2RGBA)

        # LAP Title
        loc_x, loc_y = XYPixelLoc.F1_MIDWAY_LAP_TEXT_LOC.value
        lap_txt = "LAP"
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=lap_txt,
                                                   loc=(loc_x, loc_y), font=self.formula1_display_wide_12px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # LAP counter
        loc_x, loc_y = XYPixelLoc.F1_MIDWAY_LAP_COUNTER_LOC.value
        current_lap = min(int(mp4_video_metrics_info[self.racecar_index].lap_counter) + 1, int(self.total_laps))
        lap_counter_text = "{} / {}".format(current_lap, int(self.total_laps))
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=lap_counter_text,
                                                   loc=(loc_x, loc_y), font=self.formula1_display_bold_16px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # Each racers ranking, name and gap time
        rank_loc_x, rank_loc_y = XYPixelLoc.F1_MIDWAY_LEADER_RANK_LOC.value
        color_code_loc_x, color_code_loc_y = XYPixelLoc.F1_MIDWAY_LEADER_COLOR_CODE_LOC.value
        display_name_loc_x, display_name_loc_y = XYPixelLoc.F1_MIDWAY_LEADER_DISPLAY_NAME_LOC.value
        gap_loc_x, gap_loc_y = XYPixelLoc.F1_MIDWAY_LEADER_GAP_LOC.value

        # All the other racers name and gap
        for i, val in enumerate(rank_name_gap_time):
            rank, display_name, gap_time, racer_number = val
            # Rank
            racer_rank = "{}".format(rank)
            major_cv_image = utils.write_text_on_image(image=major_cv_image, text=racer_rank,
                                                       loc=(rank_loc_x, rank_loc_y),
                                                       font=self.formula1_display_regular_12px,
                                                       font_color=RaceCarColorToRGB.White.value,
                                                       font_shadow_color=RaceCarColorToRGB.Black.value)
            # Draw racer color code icon
            major_cv_image = utils.plot_rectangular_image_on_main_image(major_cv_image,
                                                                        self._racer_color_code_rect_img[racer_number],
                                                                        (color_code_loc_x, color_code_loc_y))
            # Adding display name to the table
            display_name_txt = display_name if len(display_name) <= 6 else "{}".format(display_name[:6])
            major_cv_image = utils.write_text_on_image(image=major_cv_image, text=display_name_txt,
                                                       loc=(display_name_loc_x, display_name_loc_y),
                                                       font=self.formula1_display_regular_12px,
                                                       font_color=RaceCarColorToRGB.White.value,
                                                       font_shadow_color=RaceCarColorToRGB.Black.value)
            # Adding gap to the table (Do not write gap for leader)
            if i:
                gap_time_text = "+{:.3f}".format(gap_time)
                major_cv_image = utils.write_text_on_image(image=major_cv_image, text=gap_time_text,
                                                           loc=(gap_loc_x, gap_loc_y),
                                                           font=self.formula1_display_regular_12px,
                                                           font_color=RaceCarColorToRGB.White.value,
                                                           font_shadow_color=RaceCarColorToRGB.Black.value)
            rank_loc_y += 20
            color_code_loc_y += 20
            display_name_loc_y += 20
            gap_loc_y += 20

        return cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2BGRA)

    def _race_finish_display(self, major_cv_image, rank_name_gap_time):
        """ This displays the stats of all the racers who have finished the lap. When the car
        finishes the lap the finisher racers get added into this list.

        Arguments:
            major_cv_image (Image): Main camera image for the racecar
            rank_name_gap_time (list): Sorted list based on the ranking along with rank, name, gap time, racer_number

        Returns:
            major_cv_image (Image): Edited Main camera image
        """
        # Applying gradient to whole major image and then writing text
        major_cv_image = utils.apply_gradient(major_cv_image, self.gradient_finisher_alpha_rgb_mul,
                                              self.one_minus_gradient_finisher_alpha)
        major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_BGR2RGBA)

        # F1 logo
        # loc_x, loc_y = XYPixelLoc.F1_LOGO_LOC.value
        # f1_logo_image = utils.get_image(TrackAssetsIconographicPngs.F1_LOGO_PNG.value,
        #                                 IconographicImageSize.F1_LOGO_IMAGE_SIZE.value,
        #                                 is_rgb=True)
        # major_cv_image = utils.plot_rectangular_image_on_main_image(major_cv_image, f1_logo_image, (loc_x, loc_y))

        # Finish Title
        loc_x, loc_y = XYPixelLoc.F1_FINISHED_TITLE_LOC.value
        finisher_txt = "Finishers"
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=finisher_txt,
                                                   loc=(loc_x, loc_y), font=self.formula1_display_bold_16px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)

        rank_loc_x, rank_loc_y = XYPixelLoc.F1_FINISHED_RANK_LOC.value
        color_code_loc_x, color_code_loc_y = XYPixelLoc.F1_FINISHED_COLOR_CODE_LOC.value
        display_name_loc_x, display_name_loc_y = XYPixelLoc.F1_FINISHED_DISPLAY_NAME_LOC.value

        for rank, display_name, _, racer_number in rank_name_gap_time:
            if racer_number not in self._finished_lap_time:
                continue

            # All racers name and gap
            racer_rank = "{}".format(rank)
            major_cv_image = utils.write_text_on_image(image=major_cv_image, text=racer_rank,
                                                       loc=(rank_loc_x, rank_loc_y),
                                                       font=self.formula1_display_regular_12px,
                                                       font_color=RaceCarColorToRGB.White.value,
                                                       font_shadow_color=RaceCarColorToRGB.Black.value)
            # Draw racer color code icon
            major_cv_image = utils.plot_rectangular_image_on_main_image(major_cv_image,
                                                                        self._racer_color_code_rect_img[racer_number],
                                                                        (color_code_loc_x, color_code_loc_y))

            # Adding display name to the table
            display_name_txt = display_name if len(display_name) <= 6 else "{}".format(display_name[:6])
            major_cv_image = utils.write_text_on_image(image=major_cv_image, text=display_name_txt,
                                                       loc=(display_name_loc_x, display_name_loc_y),
                                                       font=self.formula1_display_regular_12px,
                                                       font_color=RaceCarColorToRGB.White.value,
                                                       font_shadow_color=RaceCarColorToRGB.Black.value)
            rank_loc_y += 20
            color_code_loc_y += 20
            display_name_loc_y += 20
        return cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2BGRA)

    def _get_racers_metric_info(self, mp4_video_metrics_info):
        racers_ranking = OrderedDict()
        is_finish_lap_time_changed = False
        for i in range(len(self.racecars_info)):
            racer_number = int(self.racecars_info[i]['name'].split("_")[1])
            mp4_video_metrics = mp4_video_metrics_info[racer_number]
            if mp4_video_metrics.done and mp4_video_metrics.lap_counter + 1 >= int(self.total_laps):
                if racer_number not in self._finished_lap_time:
                    self._finished_lap_time[racer_number] = mp4_video_metrics.total_evaluation_time
                    is_finish_lap_time_changed = True
            else:
                racers_ranking[racer_number] = mp4_video_metrics.completion_percentage / 100.0 +\
                    mp4_video_metrics.lap_counter

        if is_finish_lap_time_changed:
            # Re-order the self._finished_lap_time ONLY if there is any information updated.
            self._finished_lap_time = OrderedDict(sorted(self._finished_lap_time.items(), key=lambda item: item[1]))

        racers_ranking = OrderedDict(sorted(racers_ranking.items(), key=lambda item: item[1], reverse=True))
        rank_name_gap_time = self._racers_rank_name_gap_time(racers_ranking, mp4_video_metrics_info)
        return racers_ranking, rank_name_gap_time

    def _edit_major_cv_image(self, major_cv_image, mp4_video_metrics_info):
        """ Apply all the editing for the Major 45degree camera image
        Args:
            major_cv_image (Image): Image straight from the camera
            mp4_video_metrics_info (list): All the racers metric information
        Returns:
            Image: Edited main camera image
        """
        racers_ranking, rank_name_gap_time = self._get_racers_metric_info(mp4_video_metrics_info)

        # Check if the done flag is set and set the banner appropriately
        cur_racer_metrics_info = mp4_video_metrics_info[self.racecar_index]
        current_lap = min(int(cur_racer_metrics_info.lap_counter) + 1, int(self.total_laps))

        agent_done = cur_racer_metrics_info.done and (current_lap == int(self.total_laps))

        if cur_racer_metrics_info.completion_percentage > 50.0 and cur_racer_metrics_info.completion_percentage < 60.0:
            major_cv_image = self._midway_racers_progress_display(major_cv_image, rank_name_gap_time,
                                                                  mp4_video_metrics_info)
            major_cv_image = self._basic_racer_info_display(major_cv_image, racers_ranking, cur_racer_metrics_info)
        elif agent_done:
            major_cv_image = self._race_finish_display(major_cv_image, rank_name_gap_time)
            major_cv_image = self._basic_racer_info_display(major_cv_image, racers_ranking, cur_racer_metrics_info,
                                                            show_racer_name=False)
        else:
            major_cv_image = self._default_edit(major_cv_image)
            major_cv_image = self._basic_racer_info_display(major_cv_image, racers_ranking, cur_racer_metrics_info)

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

        # Track image is placed at the bottom right with some offset (only in leaderboard tracks)
        x_min = -(self.track_loc_offset[1] + track_icongraphy_scaled.shape[0])
        x_max = gradient_img.shape[0] - self.track_loc_offset[1]
        y_min = -(self.track_loc_offset[0] + track_icongraphy_scaled.shape[1])
        y_max = gradient_img.shape[1] - self.track_loc_offset[0]

        # This is used as the offset for plotting the agent dots
        self.track_start_loc = (gradient_img.shape[1] + y_min, gradient_img.shape[0] + x_min)

        for channel in range(0, 4):
            gradient_img[x_min:x_max, y_min:y_max, channel] =\
                (track_icongraphy_alpha * track_icongraphy_scaled[:, :, channel]) + \
                (1 - track_icongraphy_alpha) * (gradient_img[x_min:x_max, y_min:y_max, channel])
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
        mp4_video_metrics_info = metric_info[FrameQueueData.AGENT_METRIC_INFO.value]
        # Find max total_evaluation_time from mp4_video_metrics_info
        # to synchronize the total evaluation time throughout racers.
        max_total_evaluation_time = max([item.total_evaluation_time for item in mp4_video_metrics_info])
        for info in mp4_video_metrics_info:
            info.total_evaluation_time = max_total_evaluation_time
        major_cv_image = self._edit_major_cv_image(major_cv_image, mp4_video_metrics_info)
        major_cv_image = self._plot_agents_on_major_cv_image(major_cv_image, mp4_video_metrics_info)
        return cv2.cvtColor(major_cv_image, cv2.COLOR_BGRA2RGB)

    def _edit_top_camera_image_util(self, top_camera_image, rank_name_gap_time, mp4_video_metrics_info):
        """ Showing stats on the top view camera image

        Arguments:
            major_cv_image (Image): Main camera image for the racecar
            rank_name_gap_time (list): Sorted list based on the ranking along with rank, name, gap time, racer_number
            mp4_video_metrics_info (list): All the racers metric information

        Returns:
            major_cv_image (Image): Edited Main camera image
        """
        # Applying gradient to whole major image and then writing text
        top_camera_image = utils.apply_gradient(top_camera_image, self.gradient_top_camera_alpha_rgb_mul,
                                                self.one_minus_gradient_top_camera_alpha)
        top_camera_image = cv2.cvtColor(top_camera_image, cv2.COLOR_BGR2RGBA)

        # LAP Title
        loc_x, loc_y = XYPixelLoc.F1_TOP_CAMERA_LAP_TEXT_LOC.value
        lap_txt = "LAP"
        top_camera_image = utils.write_text_on_image(image=top_camera_image, text=lap_txt,
                                                     loc=(loc_x, loc_y), font=self.formula1_display_wide_12px,
                                                     font_color=RaceCarColorToRGB.White.value,
                                                     font_shadow_color=RaceCarColorToRGB.Black.value)

        # LAP counter
        loc_x, loc_y = XYPixelLoc.F1_TOP_CAMERA_LAP_COUNTER_LOC.value
        # For Top view, lap counter should be based on leader's lap count.
        leader_index = rank_name_gap_time[0][3]
        current_lap = min(int(mp4_video_metrics_info[leader_index].lap_counter) + 1, int(self.total_laps))
        lap_counter_text = "{} / {}".format(current_lap, int(self.total_laps))
        top_camera_image = utils.write_text_on_image(image=top_camera_image, text=lap_counter_text,
                                                     loc=(loc_x, loc_y), font=self.formula1_display_bold_16px,
                                                     font_color=RaceCarColorToRGB.White.value,
                                                     font_shadow_color=RaceCarColorToRGB.Black.value)
        # Each racers ranking, name and gap time
        rank_loc_x, rank_loc_y = XYPixelLoc.F1_TOP_CAMERA_LEADER_RANK_LOC.value
        color_code_loc_x, color_code_loc_y = XYPixelLoc.F1_TOP_CAMERA_LEADER_COLOR_CODE_LOC.value
        display_name_loc_x, display_name_loc_y = XYPixelLoc.F1_TOP_CAMERA_LEADER_DISPLAY_NAME_LOC.value
        gap_loc_x, gap_loc_y = XYPixelLoc.F1_TOP_CAMERA_LEADER_GAP_LOC.value

        for row in range(4):
            for col in range(3):
                if (row + 4 * col) >= len(rank_name_gap_time):
                    continue
                rank, display_name, gap_time, racer_number = rank_name_gap_time[row + 4 * col]
                # Rank
                loc_x, loc_y = rank_loc_x + (col * 185), rank_loc_y + (row * 20)
                racer_rank = "{}".format(rank)
                top_camera_image = utils.write_text_on_image(image=top_camera_image, text=racer_rank,
                                                             loc=(loc_x, loc_y),
                                                             font=self.formula1_display_regular_12px,
                                                             font_color=RaceCarColorToRGB.White.value,
                                                             font_shadow_color=RaceCarColorToRGB.Black.value)
                # Draw racer color code icon
                loc_x, loc_y = color_code_loc_x + (col * 185), color_code_loc_y + (row * 20)
                top_camera_image = utils.plot_rectangular_image_on_main_image(top_camera_image,
                                                                              self._racer_color_code_rect_img[racer_number],
                                                                              (loc_x, loc_y))
                # Adding display name to the table
                loc_x, loc_y = display_name_loc_x + (col * 185), display_name_loc_y + (row * 20)
                display_name_txt = display_name if len(display_name) <= 6 else "{}".format(display_name[:6])
                top_camera_image = utils.write_text_on_image(image=top_camera_image, text=display_name_txt,
                                                             loc=(loc_x, loc_y),
                                                             font=self.formula1_display_regular_12px,
                                                             font_color=RaceCarColorToRGB.White.value,
                                                             font_shadow_color=RaceCarColorToRGB.Black.value)
                # Adding gap to the table
                if row != 0 or col != 0:
                    loc_x, loc_y = gap_loc_x + (col * 180), gap_loc_y + (row * 20)
                    # The gradient box images are not equally placed for gaps
                    if col == 2:
                        loc_x, loc_y = gap_loc_x + (col * 182), gap_loc_y + (row * 20)

                    gap_time_text = "+{:.3f}".format(gap_time)
                    top_camera_image = utils.write_text_on_image(image=top_camera_image, text=gap_time_text,
                                                                 loc=(loc_x, loc_y),
                                                                 font=self.formula1_display_regular_12px,
                                                                 font_color=RaceCarColorToRGB.White.value,
                                                                 font_shadow_color=RaceCarColorToRGB.Black.value)
        return top_camera_image

    def edit_top_camera_image(self, top_camera_image, metric_info):
        """ Editing the top camera image

        Arguments:
            top_camera_image (Image): Edited top camera image
        """
        mp4_video_metrics_info = metric_info[FrameQueueData.AGENT_METRIC_INFO.value]
        _, rank_name_gap_time = self._get_racers_metric_info(mp4_video_metrics_info)
        top_camera_image = self._edit_top_camera_image_util(top_camera_image, rank_name_gap_time, mp4_video_metrics_info)
        return cv2.cvtColor(top_camera_image, cv2.COLOR_BGRA2RGB)
