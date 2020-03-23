""" Image editing class for head to bot, time-trail, obstacle where
there is only single agent
"""
import datetime
import logging
import rospy
import cv2
from sensor_msgs.msg import Image as ROSImg

from markov.utils import Logger
from markov.rospy_wrappers import ServiceProxyWrapper
from deepracer_simulation_environment.srv import VideoMetricsSrvRequest, VideoMetricsSrv
from mp4_saving import utils
from mp4_saving.constants import (RaceCarColorToRGB,
                                  IconographicImageSize,
                                  TrackAssetsIconographicPngs,
                                  RACE_TYPE_TO_VIDEO_TEXT_MAPPING)
from mp4_saving.image_editing_interface import ImageEditingInterface
from mp4_saving.top_view_graphics import TopViewGraphics

LOG = Logger(__name__, logging.INFO).get_logger()

class SingleAgentImageEditing(ImageEditingInterface):
    """ Image editing class for head to bot, time-trail, obstacle where
    there is only single agent
    """
    def __init__(self, racecar_info, race_type):
        """ Initializing the required data for the head to bot, time-trail. This is used for single agent
        Arguments:
            racecars_info (list): list of dict having information of the agent
            race_type (str): Since this class is reused for all the different race_type
        """
        self.racecar_info = racecar_info
        self.race_type = race_type
        # Store the font which we will use to write the phase with
        self.amazon_ember_regular_20px = utils.get_font('AmazonEmber-Regular', 20)
        self.amazon_ember_heavy_30px = utils.get_font('AmazonEmber-Heavy', 30)
        self.amazon_ember_light_18px = utils.get_font('AmazonEmber-Light', 18)
        self.amazon_ember_light_20px = utils.get_font('AmazonEmber-Light', 20)
        self.amazon_ember_light_italic_20px = utils.get_font('AmazonEmber-LightItalic', 20)

        # The track image as iconography
        self.track_icongraphy_img = utils.get_track_iconography_image()
        self.gradient_img = utils.get_image(TrackAssetsIconographicPngs.OBSTACLE_OVERLAY_PNG.value,
                                            IconographicImageSize.FULL_IMAGE_SIZE.value)
        self.gradient_alpha = self.gradient_img[:, :, 3] / 255.0

        # Subscribing to the agent metrics
        rospy.wait_for_service("/agent/mp4_video_metrics")
        self.mp4_video_metrics_srv = ServiceProxyWrapper("/agent/mp4_video_metrics", VideoMetricsSrv)

        # Top camera information
        top_camera_info = utils.get_top_camera_info()
        self.edited_topview_pub = rospy.Publisher('/deepracer/topview_stream', ROSImg, queue_size=1)
        self.top_view_graphics = TopViewGraphics(top_camera_info.horizontal_fov, top_camera_info.padding_pct,
                                                 top_camera_info.image_width, top_camera_info.image_height,
                                                 racecar_info)

    def _edit_major_cv_image(self, major_cv_image):
        """ Apply all the editing for the Major 45degree camera image
        Args:
            major_cv_image (Image): Image straight from the camera
        Returns:
            Image: Edited main camera image
        """
        # Applying gradient to whole major image and then writing text
        major_cv_image = utils.apply_gradient(major_cv_image, self.gradient_img, self.gradient_alpha)

        mp4_video_metrics_info = self.mp4_video_metrics_srv(VideoMetricsSrvRequest())

        # Top left location of the picture
        loc_x, loc_y = (10, 10)

        # Display name (Racer name/Model name)
        display_name = self.racecar_info['display_name']
        display_name_txt = display_name if len(display_name) < 15 else "{}...".format(display_name[:15])
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=display_name_txt,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_regular_20px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # Lap Counter
        loc_y += 30
        total_laps = rospy.get_param("NUMBER_OF_TRIALS", 0)
        current_lap = int(mp4_video_metrics_info.lap_counter) + 1
        lap_counter_text = "{}/{}".format(current_lap, total_laps)
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=lap_counter_text,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_heavy_30px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # total_evaluation_time (Race time)
        loc_y += 45
        total_eval_milli_seconds = mp4_video_metrics_info.total_evaluation_time
        time_delta = datetime.timedelta(milliseconds=total_eval_milli_seconds)
        total_eval_time_text = "Race | {}".format(utils.milliseconds_to_timeformat(time_delta))
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=total_eval_time_text,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_light_18px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # Reset counter
        loc_y += 25
        reset_counter_text = "Reset | {}".format(mp4_video_metrics_info.reset_counter)
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=reset_counter_text,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_light_18px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # Speed
        loc_y = 420
        speed_text = "{} m/s".format(utils.get_speed_formatted_str(mp4_video_metrics_info.throttle))
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=speed_text,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_light_20px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # Evaluation type
        loc_y += 25
        # TODO - Show text based on whether its a race or customer run evaluation
        race_text = "race"
        evaluation_type_txt = "{} {}".format(RACE_TYPE_TO_VIDEO_TEXT_MAPPING[self.race_type], race_text)
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=evaluation_type_txt,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_light_italic_20px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # Check if the done flag is set and set the banner appropriately
        if mp4_video_metrics_info.done and (int(total_laps) == current_lap):
            # When the cv2 text is written, it automatically drops the alpha value of the image
            major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2RGBA)
            racecomplete_image = utils.get_image(TrackAssetsIconographicPngs.RACE_COMPLETE_OVERLAY_PNG.value,
                                                 IconographicImageSize.RACE_COMPLETE_IMAGE_SIZE.value)
            x_offset = major_cv_image.shape[1] - racecomplete_image.shape[1]//2
            y_offset = major_cv_image.shape[0] - 180 - racecomplete_image.shape[0]//2
            major_cv_image = utils.plot_rectangular_image_on_main_image(
                major_cv_image, racecomplete_image, (x_offset, y_offset))
        return major_cv_image

    def _edit_minor_cv_image(self):
        """ Apply all the editing for the iconographic of the top view camera image
        Returns:
            Image: Edited image of the iconographic view of the top camera
        """
        return self.top_view_graphics.plot_agents_as_circles(self.track_icongraphy_img.copy())

    def edit_image(self, major_cv_image):
        major_cv_image = self._edit_major_cv_image(major_cv_image)
        minor_cv_image = self._edit_minor_cv_image()
        return utils.overlay_track_images(major_cv_image, minor_cv_image)
