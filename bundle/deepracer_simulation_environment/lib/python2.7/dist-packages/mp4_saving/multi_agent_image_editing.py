"""Image editing class for head to head where there are multiple agents
"""
import datetime
import logging
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ROSImg

from markov.utils import Logger
from markov.rospy_wrappers import ServiceProxyWrapper
from mp4_saving.top_view_graphics import TopViewGraphics
from deepracer_simulation_environment.srv import VideoMetricsSrvRequest, VideoMetricsSrv
from mp4_saving.constants import (RaceCarColorToRGB,
                                  RACE_TYPE_TO_VIDEO_TEXT_MAPPING,
                                  TrackAssetsIconographicPngs, IconographicImageSize)
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
        self.race_type = race_type
        # init cv bridge
        self.bridge = CvBridge()
        # Store the font which we will use to write the phase with
        self.amazon_ember_regular_20px = utils.get_font('AmazonEmber-Regular', 20)
        self.amazon_ember_heavy_30px = utils.get_font('AmazonEmber-Heavy', 30)
        self.amazon_ember_light_18px = utils.get_font('AmazonEmber-Light', 18)
        self.amazon_ember_light_20px = utils.get_font('AmazonEmber-Light', 20)
        self.amazon_ember_light_italic_20px = utils.get_font('AmazonEmber-LightItalic', 20)

        # The track image as iconography
        self.track_icongraphy_img = utils.get_track_iconography_image()
        self.gradient_img = utils.get_image(TrackAssetsIconographicPngs.HEAD_TO_HEAD_OVERLAY_PNG.value,
                                            IconographicImageSize.FULL_IMAGE_SIZE.value)
        self.gradient_alpha = self.gradient_img[:, :, 3] / 255.0

        self.mp4_video_metrics_srv_list = list()
        for racecar_info in self.racecars_info:
            agent_name = 'agent' if len(racecars_info) == 1 else "agent_{}".format(racecar_info['name'].split("_")[1])
            rospy.wait_for_service("/{}/{}".format(agent_name, "mp4_video_metrics"))
            self.mp4_video_metrics_srv_list.append(ServiceProxyWrapper(
                "/{}/{}".format(agent_name, "mp4_video_metrics"), VideoMetricsSrv))

        # Top camera information
        top_camera_info = utils.get_top_camera_info()
        self.edited_topview_pub = rospy.Publisher('/deepracer/topview_stream', ROSImg, queue_size=1)
        self.top_view_graphics = TopViewGraphics(top_camera_info.horizontal_fov, top_camera_info.padding_pct,
                                                 top_camera_info.image_width, top_camera_info.image_height,
                                                 racecars_info)

    def _edit_major_cv_image(self, major_cv_image):
        """ Apply all the editing for the Major 45degree camera image
        Args:
            major_cv_image (Image): Image straight from the camera
        Returns:
            Image: Edited main camera image
        """
        # Applying gradient to whole major image and then writing text
        major_cv_image = utils.apply_gradient(major_cv_image, self.gradient_img, self.gradient_alpha)

        # Subscribing to the agent metrics
        mp4_video_metrics_info = list()
        for racecar_info, mp4_video_metrics_srv in zip(self.racecars_info, self.mp4_video_metrics_srv_list):
            mp4_video_metrics = mp4_video_metrics_srv(VideoMetricsSrvRequest())
            mp4_video_metrics_info.append(mp4_video_metrics)

        # Adding display name to the image
        display_name_loc = [(10, 10), (450, 10)]
        agents_speed = 0
        agent_done = False
        for i, racecar_info in enumerate(self.racecars_info):
            loc_x, loc_y = display_name_loc[i][0], display_name_loc[i][1]
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
        loc_x, loc_y = 10, 420
        speed_text = "{} m/s".format(utils.get_speed_formatted_str(agents_speed))
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
        # total_evaluation_time (Race time)
        loc_x, loc_y = 240, 10
        total_eval_milli_seconds = mp4_video_metrics_info[0].total_evaluation_time
        time_delta = datetime.timedelta(milliseconds=total_eval_milli_seconds)
        total_eval_time_text = "Race | {}".format(utils.milliseconds_to_timeformat(time_delta))
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=total_eval_time_text,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_light_18px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # Check if the done flag is set and set the banner appropriately
        if agent_done:
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
        # Edit the top camera image to have circles instead of dots
        return self.top_view_graphics.plot_agents_as_circles(self.track_icongraphy_img.copy())

    def edit_image(self, major_cv_image):
        major_cv_image = self._edit_major_cv_image(major_cv_image)
        minor_cv_image = self._edit_minor_cv_image()
        return utils.overlay_track_images(major_cv_image, minor_cv_image)
