""" Each race type has its own metrics to be popullated on the image.
This module takes care of addressing each race type editting of images.
"""
import logging
import cv2
import numpy as np

from markov.log_handler.logger import Logger
from markov.utils import get_racecar_idx
from mp4_saving.top_view_graphics import TopViewGraphics
from mp4_saving.constants import (RaceCarColorToRGB, SCALE_RATIO, IconographicImageSize,
                                  XYPixelLoc, FrameQueueData)
from mp4_saving import utils
from mp4_saving.image_editing_interface import ImageEditingInterface

LOG = Logger(__name__, logging.INFO).get_logger()

class TrainingImageEditing(ImageEditingInterface):
    """ Editing the image in training phase
    """
    def __init__(self, racecar_name, racecar_info, race_type):
        """ To capture the video of evaluation done during the training phase
        Args:
            racecar_info (dict): Information of the agent
        """
        self.racecar_info = racecar_info
        racecar_index = get_racecar_idx(racecar_name)
        self.racecar_index = racecar_index if racecar_index else 0
        # Store the font which we will use to write the phase with
        self.training_phase_font = utils.get_font('Amazon_Ember_RgIt', 35)

        # The track image as iconography
        self.track_icongraphy_img = utils.get_track_iconography_image()

        # Track image offset
        self.track_loc_offset = XYPixelLoc.TRACK_IMG_WITHOUT_OFFSET_LOC.value

        # Gradient overlay image
        width, height = IconographicImageSize.FULL_IMAGE_SIZE.value
        image = np.zeros(height*width*4)
        image.resize(height, width, 4)
        self.gradient_img = self._plot_track_on_gradient(image)
        self.gradient_alpha_rgb_mul, self.one_minus_gradient_alpha = utils.get_gradient_values(self.gradient_img)

        # Top camera information
        top_camera_info = utils.get_top_camera_info()
        self.top_view_graphics = TopViewGraphics(top_camera_info.horizontal_fov, top_camera_info.padding_pct,
                                                 top_camera_info.image_width, top_camera_info.image_height,
                                                 racecar_info)

    def _edit_major_cv_image(self, major_cv_image, cur_training_phase):
        """ Apply all the editing for the Major 45degree camera image
        Args:
            major_cv_image (Image): Image straight from the camera
        Returns:
            Image: Edited main camera image
        """
        # Applying gradient to whole major image and then writing text
        major_cv_image = utils.apply_gradient(major_cv_image, self.gradient_alpha_rgb_mul,
                                              self.one_minus_gradient_alpha)

        # Add the label that lets the user know the training phase
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=cur_training_phase,
                                                   loc=XYPixelLoc.TRAINING_PHASE_LOC.value,
                                                   font=self.training_phase_font, font_color=None,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2BGRA)
        return major_cv_image

    def _plot_track_on_gradient(self, gradient_img):
        """ For the given gradient apply the track iconographic image and use this to apply gradient
        on each camera frame. Previously this was done on the top camera which changed every frame. But
        with the track iconographic image set static, adding the track on gradient is more optimized.

        Arguments:
            gradient_img (str): Gradient image

        Returns:
            (Image): Edited gradient image with track image
        """
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
            Image: Edited image of the iconographic view of the top camera
        """
        agents_loc = [(metric.x, metric.y) for metric in mp4_video_metrics_info]
        objects_loc = []
        if mp4_video_metrics_info[0].object_locations:
            objects_loc = [(object_loc.x, object_loc.y) for object_loc in mp4_video_metrics_info[0].object_locations]
        return self.top_view_graphics.plot_agents_as_circles(
            major_cv_image, agents_loc, objects_loc, self.track_start_loc)

    def edit_image(self, major_cv_image, metric_info):
        mp4_video_metrics_info = metric_info[FrameQueueData.AGENT_METRIC_INFO.value]
        cur_training_phase = metric_info[FrameQueueData.TRAINING_PHASE.value]

        major_cv_image = self._edit_major_cv_image(major_cv_image, cur_training_phase)
        major_cv_image = self._plot_agents_on_major_cv_image(major_cv_image, mp4_video_metrics_info)
        return cv2.cvtColor(major_cv_image, cv2.COLOR_BGRA2RGB)
