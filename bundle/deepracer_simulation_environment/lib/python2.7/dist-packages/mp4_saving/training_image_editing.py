""" Each race type has its own metrics to be popullated on the image.
This module takes care of addressing each race type editting of images.
"""
import logging
import cv2
import rospy
import rospkg
from sensor_msgs.msg import Image as ROSImg
from std_msgs.msg import String

from markov.log_handler.logger import Logger
from mp4_saving.top_view_graphics import TopViewGraphics
from mp4_saving.constants import (RaceCarColorToRGB, SCALE_RATIO, IconographicImageSize,
                                  TrackAssetsIconographicPngs, XYPixelLoc)
from mp4_saving import utils
from mp4_saving.image_editing_interface import ImageEditingInterface

LOG = Logger(__name__, logging.INFO).get_logger()

class TrainingImageEditing(ImageEditingInterface):
    """ Editing the image in training phase
    """
    def __init__(self, racecar_info):
        """ To capture the video of evaluation done during the training phase
        Args:
            racecar_info (dict): Information of the agent
        """
        self.racecar_info = racecar_info
        # Store the font which we will use to write the phase with
        self.training_phase_font = utils.get_font('Amazon_Ember_RgIt', 35)

        # Subscriber to get the phase of the training (Ideal, training, evaluation)
        rospy.Subscriber('/agent/training_phase', String, self._training_phase_cb_)

        # The track image as iconography
        self.track_icongraphy_img = utils.get_track_iconography_image()
        #
        # TODO: Currently dont have the gradient image for the training. So not using any gradient overlay on image
        # self.gradient_img = utils.get_image(TrackAssetsIconographicPngs.OBSTACLE_OVERLAY_PNG.value,
        #                                     IconographicImageSize.FULL_IMAGE_SIZE.value)
        # self.gradient_alpha = self.gradient_img[:, :, 3] / 255.0
        #

        # String indicating the current phase
        self._current_training_phase = 'Initializing'

        # Top camera information
        top_camera_info = utils.get_top_camera_info()
        self.edited_topview_pub = rospy.Publisher('/deepracer/topview_stream', ROSImg, queue_size=1)
        self.top_view_graphics = TopViewGraphics(top_camera_info.horizontal_fov, top_camera_info.padding_pct,
                                                 top_camera_info.image_width, top_camera_info.image_height,
                                                 racecar_info)

    def _training_phase_cb_(self, phase):
        """ Callback function that gives the training phase - Whether its in
        evaluation, ideal, training, initializing
        Args:
            phase: [description]
        """
        self._current_training_phase = phase.data

    def _edit_major_cv_image(self, major_cv_image):
        """ Apply all the editing for the Major 45degree camera image
        Args:
            major_cv_image (Image): Image straight from the camera
        Returns:
            Image: Edited main camera image
        """
        #
        # TODO: Currently dont have the gradient image for the training. So not using any gradient overlay on image
        # major_cv_image = utils.apply_gradient(major_cv_image, self.gradient_img, self.gradient_alpha)
        #
        if self._current_training_phase:
            # Add the label that lets the user know the training phase
            major_cv_image = utils.write_text_on_image(image=major_cv_image, text=self._current_training_phase,
                                                       loc=XYPixelLoc.TRAINING_PHASE_LOC.value,
                                                       font=self.training_phase_font, font_color=None,
                                                       font_shadow_color=RaceCarColorToRGB.Black.value)
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
