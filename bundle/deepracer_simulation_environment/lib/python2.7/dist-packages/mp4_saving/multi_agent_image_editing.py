"""Image editing class for head to head where there are multiple agents
"""
import datetime
import logging
import rospy
import cv2

from markov.log_handler.logger import Logger
from markov.reset.constants import RaceType
from markov.utils import get_racecar_idx
from mp4_saving.top_view_graphics import TopViewGraphics
from mp4_saving.constants import (RaceCarColorToRGB,
                                  RACE_TYPE_TO_VIDEO_TEXT_MAPPING, SCALE_RATIO,
                                  TrackAssetsIconographicPngs, IconographicImageSize,
                                  XYPixelLoc, AWS_DEEPRACER_WATER_MARK, RACE_COMPLETE_Y_OFFSET,
                                  FrameQueueData)
from mp4_saving import utils
from mp4_saving.image_editing_interface import ImageEditingInterface

LOG = Logger(__name__, logging.INFO).get_logger()

class MultiAgentImageEditing(ImageEditingInterface):
    """Image editing class for head to head where there are multiple agents
    """
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
        self.amazon_ember_regular_20px = utils.get_font('AmazonEmber-Regular', 20)
        self.amazon_ember_regular_16px = utils.get_font('AmazonEmber-Regular', 16)
        self.amazon_ember_heavy_30px = utils.get_font('AmazonEmber-Heavy', 30)
        self.amazon_ember_light_18px = utils.get_font('AmazonEmber-Light', 18)
        self.amazon_ember_light_20px = utils.get_font('AmazonEmber-Light', 20)
        self.amazon_ember_light_italic_20px = utils.get_font('AmazonEmber-LightItalic', 20)

        self.is_racing = rospy.get_param("VIDEO_JOB_TYPE", "") == "RACING"
        self.is_league_leaderboard = rospy.get_param("LEADERBOARD_TYPE", "") == "LEAGUE"
        self.leaderboard_name = rospy.get_param("LEADERBOARD_NAME", "")

        # The track image as iconography
        self.track_icongraphy_img = utils.get_track_iconography_image()

        # Track image offset
        self.track_loc_offset = XYPixelLoc.TRACK_IMG_WITH_OFFSET_LOC.value if self.is_league_leaderboard \
            else XYPixelLoc.TRACK_IMG_WITHOUT_OFFSET_LOC.value

        gradient_img_path = TrackAssetsIconographicPngs.HEAD_TO_HEAD_OVERLAY_PNG_LEAGUE_LEADERBOARD.value \
            if self.is_league_leaderboard else TrackAssetsIconographicPngs.HEAD_TO_HEAD_OVERLAY_PNG.value
        self.gradient_img = self._plot_track_on_gradient(gradient_img_path)
        self.gradient_alpha_rgb_mul, self.one_minus_gradient_alpha = utils.get_gradient_values(self.gradient_img)

        # Top camera information
        top_camera_info = utils.get_top_camera_info()
        self.top_view_graphics = TopViewGraphics(top_camera_info.horizontal_fov, top_camera_info.padding_pct,
                                                 top_camera_info.image_width, top_camera_info.image_height,
                                                 racecars_info)

    def _edit_major_cv_image(self, major_cv_image, mp4_video_metrics_info):
        """ Apply all the editing for the Major 45degree camera image
        Args:
            major_cv_image (Image): Image straight from the camera
        Returns:
            Image: Edited main camera image
        """
        # Applying gradient to whole major image and then writing text
        major_cv_image = utils.apply_gradient(major_cv_image, self.gradient_alpha_rgb_mul,
                                              self.one_minus_gradient_alpha)

        # Adding display name to the image
        agents_speed = 0
        agent_done = False
        if self.race_type != RaceType.F1.value:
            for i, racecar_info in enumerate(self.racecars_info):
                loc_x, loc_y = XYPixelLoc.MULTI_AGENT_DISPLAY_NAME_LOC.value[i][0], XYPixelLoc.MULTI_AGENT_DISPLAY_NAME_LOC.value[i][1]
                # Display name (Racer name/Model name)
                display_name = racecar_info['display_name']
                display_name_txt = display_name if len(display_name) < 15 else "{}...".format(display_name[:15])
                major_cv_image = utils.write_text_on_image(image=major_cv_image, text=display_name_txt,
                                                           loc=(loc_x, loc_y), font=self.amazon_ember_regular_20px,
                                                           font_color=RaceCarColorToRGB.White.value,
                                                           font_shadow_color=RaceCarColorToRGB.Black.value)
                # Lap Counter
                loc_y += 30
                total_laps = rospy.get_param("NUMBER_OF_TRIALS", 0)
                current_lap = int(mp4_video_metrics_info[i].lap_counter) + 1
                lap_counter_text = "{}/{}".format(current_lap, total_laps)
                major_cv_image = utils.write_text_on_image(image=major_cv_image, text=lap_counter_text,
                                                           loc=(loc_x, loc_y), font=self.amazon_ember_heavy_30px,
                                                           font_color=RaceCarColorToRGB.White.value,
                                                           font_shadow_color=RaceCarColorToRGB.Black.value)
                # Reset counter
                loc_y += 45
                reset_counter_text = "Reset | {}".format(mp4_video_metrics_info[i].reset_counter)
                major_cv_image = utils.write_text_on_image(image=major_cv_image, text=reset_counter_text,
                                                           loc=(loc_x, loc_y), font=self.amazon_ember_light_18px,
                                                           font_color=RaceCarColorToRGB.White.value,
                                                           font_shadow_color=RaceCarColorToRGB.Black.value)
                if self.racecar_name == racecar_info['name']:
                    agents_speed = mp4_video_metrics_info[i].throttle
                # The race is complete when total lap is same as current lap and done flag is set
                agent_done = agent_done or (mp4_video_metrics_info[i].done and (current_lap == int(total_laps)))

        # Speed
        loc_x, loc_y = XYPixelLoc.SPEED_EVAL_LOC.value
        if self.is_league_leaderboard:
            loc_x, loc_y = XYPixelLoc.SPEED_LEADERBOARD_LOC.value
        speed_text = "{} m/s".format(utils.get_speed_formatted_str(agents_speed))
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=speed_text,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_light_20px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # Leaderboard name
        if self.is_league_leaderboard:
            major_cv_image = utils.write_text_on_image(image=major_cv_image, text=self.leaderboard_name,
                                                       loc=XYPixelLoc.LEADERBOARD_NAME_LOC.value,
                                                       font=self.amazon_ember_regular_16px,
                                                       font_color=RaceCarColorToRGB.White.value,
                                                       font_shadow_color=RaceCarColorToRGB.Black.value)

        # Evaluation type
        loc_x, loc_y = XYPixelLoc.RACE_TYPE_EVAL_LOC.value
        if self.is_league_leaderboard:
            loc_x, loc_y = XYPixelLoc.RACE_TYPE_RACE_LOC.value
        race_text = "race" if self.is_racing else "evaluation"
        evaluation_type_txt = "{} {}".format(RACE_TYPE_TO_VIDEO_TEXT_MAPPING[self.race_type], race_text)
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=evaluation_type_txt,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_light_italic_20px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # total_evaluation_time (Race time)
        loc_x, loc_y = XYPixelLoc.MULTI_AGENT_EVAL_TIME.value
        total_eval_milli_seconds = mp4_video_metrics_info[0].total_evaluation_time
        time_delta = datetime.timedelta(milliseconds=total_eval_milli_seconds)
        total_eval_time_text = "Race | {}".format(utils.milliseconds_to_timeformat(time_delta))
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=total_eval_time_text,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_light_18px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # AWS Deepracer logo at the bottom for the community leaderboard
        if self.is_league_leaderboard:
            major_cv_image = utils.write_text_on_image(image=major_cv_image,
                                                       text=AWS_DEEPRACER_WATER_MARK,
                                                       loc=XYPixelLoc.AWS_DEEPRACER_WATER_MARK_LOC.value,
                                                       font=self.amazon_ember_regular_16px,
                                                       font_color=RaceCarColorToRGB.White.value,
                                                       font_shadow_color=RaceCarColorToRGB.Black.value)

        # Check if the done flag is set and set the banner appropriately
        if agent_done:
            # When the cv2 text is written, it automatically drops the alpha value of the image
            rel_y_offset = XYPixelLoc.TRACK_IMG_WITH_OFFSET_LOC.value[1] if self.is_league_leaderboard else 0
            major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2RGBA)
            racecomplete_image = utils.get_image(TrackAssetsIconographicPngs.RACE_COMPLETE_OVERLAY_PNG.value,
                                                 IconographicImageSize.RACE_COMPLETE_IMAGE_SIZE.value)
            x_offset = major_cv_image.shape[1] - racecomplete_image.shape[1]//2
            y_offset = major_cv_image.shape[0] - RACE_COMPLETE_Y_OFFSET - rel_y_offset - racecomplete_image.shape[0]//2
            major_cv_image = utils.plot_rectangular_image_on_main_image(
                major_cv_image, racecomplete_image, (x_offset, y_offset))
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
        track_icongraphy_alpha = track_icongraphy_scaled[:, :, 3]/255.0

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
        major_cv_image = self._edit_major_cv_image(major_cv_image, mp4_video_metrics_info)
        major_cv_image = self._plot_agents_on_major_cv_image(major_cv_image, mp4_video_metrics_info)
        return cv2.cvtColor(major_cv_image, cv2.COLOR_BGRA2RGB)
