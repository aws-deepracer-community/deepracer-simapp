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

""" For the Head to head racing to identify which car is leading  and to provide enough
information of the two cars some kind of marking is required. To solve this
a bigger circle is made to move along the track to easily identify the agent location.
This class takes the top view of the track and overlaps the agents with different color
circles by getting the agents (x,y) value.
"""
import math
import logging
import numpy as np
import rospy
import cv2

from markov.log_handler.logger import Logger
from markov.reset.constants import RaceType
from markov.track_geom.track_data import TrackData
from markov.utils import force_list
from mp4_saving.constants import (TrackAssetsIconographicPngs,
                                  IconographicImageSize, SCALE_RATIO,
                                  ModelImgNames, XYPixelLoc)
from mp4_saving import utils

LOG = Logger(__name__, logging.INFO).get_logger()

class TopViewGraphics(object):
    """ For the Head to head racing to identify which car is leading  and to provide enough
    information of the two cars some kind of marking is required. To solve this
    a bigger circle is made to move along the track to easily identify the agent location.
    This class takes the top view of the track and overlaps the agents with different color
    circles by getting the agents (x,y) value.
    The camera location, horizontal fov, padding percentage, image width and height are
    taken from the top view camera.
    """
    def __init__(self, horizontal_fov, padding_percent, image_width, image_height, racecars_info,
                 race_type=None,
                 is_virtual_event=False):
        """ Camera information is required to map the (x, y) value of the agent on the camera image.
        This is because each track will have its own FOV and padding percent because to avoid
        Z-fighting. Once the camera position, padding percentage is available. We can map the
        (x,y) of the agent with respect to the track to a new plane of the camera image with
        padding.

        Arguments:
            horizontal_fov (float): Horizontal field of view of the camera for the given track
            padding_percent (float): The padding percentage to cover the whole track and look pretty
            image_width (int): Image width from the camera
            image_height (int): Image width from the camera
            racecars_info (list): This is the list of dicts of racecars on the track
            race_type (str): Type of the race, TT, H2B, OA, F1. (default: {None})
            is_virtual_event (bool): True if virtual event and False otherwise.
        """
        self.horizontal_fov = horizontal_fov
        self.padding_percent = padding_percent
        self.image_width = image_width
        self.image_height = image_height
        self.racecars_info = force_list(racecars_info)
        self.is_f1_race_type = race_type == RaceType.F1.value
        self._is_virtual_event = is_virtual_event

        self.model_imgs = self._get_all_models_info()

        # Track information is required get the track bounds (xmin, xmax, ymin, ymax)
        # Also the agent location for that instant
        self.track_data = TrackData.get_instance()
        # This maps the (x, y) of an agent in track frame to the top camera image frame
        self.initialize_parameters()

    def _get_all_models_info(self):
        """ This function is used to gather all the agents, obstacles, bots
        spawn on the track. So that the exact location is gathered and icons are drawn
        overlaying on top of it. Thus making it prominent.

        Returns:
            (dict): "object_imgs" - Appropriate image icons for boxes and bots
                    "agent_imgs" - Appropriate image icons for agents
                    "agent_num_imgs" - Appropriate image icons for agents number
        """
        object_imgs = list()
        agent_imgs = list()
        agent_num_imgs = list()
        # Adding obstacles to the list
        num_obstacles = int(rospy.get_param("NUMBER_OF_OBSTACLES", 0))
        if num_obstacles:
            # Other agents also come as object_locations so first plot all the obstacles and
            # Then overlay agents on top.
            for i in range(num_obstacles):
                object_imgs.append(utils.get_image(TrackAssetsIconographicPngs.OBSTACLES_PNG.value,
                                                   IconographicImageSize.OBSTACLE_IMAGE_SIZE.value))

        # Adding bot cars to the list
        num_bots = int(rospy.get_param("NUMBER_OF_BOT_CARS", 0))
        if num_bots:
            # Other agents also come as object_locations so first plot all the obstacles and
            # Then overlay agents on top.
            for i in range(num_bots):
                object_imgs.append(utils.get_image(TrackAssetsIconographicPngs.BOTS_PNG.value,
                                                   IconographicImageSize.BOT_CAR_IMAGE_SIZE.value))

        # Adding all agents to the list
        for i, _ in enumerate(self.racecars_info):
            # If the number of racecars are greater than 2
            if self.is_f1_race_type:
                hex_car_color = self.racecars_info[i]['racecar_color'].split('_')[-1]
                racer_number = int(self.racecars_info[i]['name'].split("_")[1])
                agents_round_img = "{}_{}".format(TrackAssetsIconographicPngs.F1_AGENTS_PNG.value,
                                                  hex_car_color)
                agent_imgs.append(utils.get_image(agents_round_img,
                                                  IconographicImageSize.F1_AGENTS_IMAGE_SIZE.value))
                agent_number_img_name = "{}_{}".format(TrackAssetsIconographicPngs.F1_AGENTS_NUM_PNG.value,
                                                       racer_number + 1)
                agent_num_imgs.append(utils.get_image(agent_number_img_name,
                                                      IconographicImageSize.F1_AGENTS_IMAGE_SIZE.value))
            else:
                agent_imgs.append(utils.get_image(TrackAssetsIconographicPngs.AGENTS_PNG.value[i % 2],
                                                  IconographicImageSize.VIRTUAL_EVENT_AGENTS_IMAGE_SIZE.value))
        return {
            ModelImgNames.OBJECT_IMGS.value: object_imgs,
            ModelImgNames.AGENT_IMGS.value: agent_imgs,
            ModelImgNames.AGENT_NUM_IMGS.value: agent_num_imgs
        }

    def initialize_parameters(self):
        """ This maps the (x, y) of an agent in track frame to the top camera image frame

        Procedure followed to map the track frame to top camera frame
        1. Get the track bounds. This will give me the min and max (x, y) of the track

        2. Now find the camera height(camera_z) by using the right angeled traiangle geometry
            i. tan(horizontal_FOV/2) = (0.5 * track_distance_x)/camera_z (Same for y track distance)

        3. Now find the extra buffer x_buf and y_buf. This is also using right angled triangle
            i. tan(horizontal_FOV/2) = (0.5 * track_distance_x + x_buf)/camera_z (Same for y track distance)

        4. The cv2 image co-ordinate plane (x_min, y_min) from camera is at the top and (x_max, y_max) at
            bottom right corner.
            i. So using the affine transformation where the y-axis is flipped
            [[1,  0, 0],
             [0, -1, 0],
             [0,  0, 1]]
            ii. But there is translatory matrix transformation.
            [[1,  0, x_buf],
             [0, -1, y_buf+track_y_dist],
             [0,  0, 1]]
            One can easily verify with this simple intution.
            Lets assume the track boundary with rectangle. Now the image from the camera is the bigger
            rectangle and its a superset of the track boundary.
            So the point on the track boundary say (x, y). Let the camera plane (X, Y)
                i. Now along the x-axis its simple by adding (x_buf + x)
                ii. For the y-axis since the y-axis is reversed. (y_buf + y_total_track_dist - y)

        5. Also the x,y of the track has negative values hence adding the x_offset, y_offset to make
            all in one co-ordinate plane

        6. The image plane is in pixel units (640, 360). So the (x, y) units must be converted to
            pixel scale.
            i. We know that the complete track distance is
                x_buf + x_track_distance + x_buf
            Simple analogy,
            For total image distance (x_buf + x_track_distance + x_buf), the value is X
            For total 640 pixel - What is the value of x'pixel
            = (640 * X)/(x_buf + x_track_distance + x_buf)

        Most of the values are pre-computed so when agent (x, y) is received, then we just multiply the
        matrices
        """
        # Track co-ordinates
        track_x_min, track_y_min, track_x_max, track_y_max = self.track_data.outer_border.bounds

        # Getting the aspect ratio
        aspect_ratio = self.image_width/self.image_height

        # Computing the horizontal and vertical fov scale to compute the camera height
        horizontal_fov_scale = 0.5 / math.tan(0.5 * self.horizontal_fov)
        vertical_fov_scale = horizontal_fov_scale * aspect_ratio
        vertical_fov = math.atan(0.5/vertical_fov_scale)/0.5

        # Total track distance
        track_x_dist = track_x_max - track_x_min
        track_y_dist = track_y_max - track_y_min

        # Camera height considering the max track dist
        camera_max_base_dist = max(horizontal_fov_scale * track_x_dist, vertical_fov_scale * track_y_dist)
        camera_z = camera_max_base_dist * (1 + self.padding_percent)

        # Extra padding added to the track
        x_buf = math.tan(self.horizontal_fov/2) * (camera_z) - (track_x_dist/2)
        y_buf = math.tan(vertical_fov/2) * (camera_z) - (track_y_dist/2)

        # Affine transformation from track plane to camera plane where y-axis in reversed
        self.gui_affine_transform = np.array([[1,  0, x_buf],
                                              [0, -1, y_buf+track_y_dist],
                                              [0,  0, 1]])

        # Converting the track units to pixel
        self.pixel_scale = np.array([self.image_width/((track_x_dist) + 2 * x_buf),
                                     self.image_height/((track_y_dist) + 2 * y_buf),
                                     1])
        #
        # The logic made to work with the outter boundries minimum values of track waypoints
        # are at the origin. This is the reason why where some track worked well not the others
        # The track that worked whose min boundary values were both negative.
        # Hence making all the min boundary values to 0 by adding/subtracting appropriately.
        #
        self.x_offset, self.y_offset = -track_x_min, -track_y_min

    def get_model_imgs_on_track_loc(self, object_loc, track_start_loc):
        """ Given the co-ordinates of agents in Top view, find its corresponding x, y location
        in the picture in picture image.

        Args:
            object_loc (float): Location of object in top view image
            track_start_loc (tuple): (x, y) offset of track image start location on the background image

        Returns:
            (tuple):
                x (int): Pixel x value on the background image
                y (int): Pixel y value on the background image
        """
        object_xy = np.array([object_loc[0] + self.x_offset,
                              object_loc[1] + self.y_offset,
                              1]).reshape(3, 1)
        gui_xy = np.dot(self.gui_affine_transform, object_xy)
        pixel_xy = gui_xy * self.pixel_scale
        loc_x, loc_y = (pixel_xy[0][0] // SCALE_RATIO + track_start_loc[0],
                        pixel_xy[1][0] // SCALE_RATIO + track_start_loc[1])
        return loc_x, loc_y

    def plot_agents_as_circles(self, image, agents_loc, objects_loc, track_start_loc):
        """
        For the given image plot a circle on top of the racecar. To plot the circle,
        fetch the current (x, y) value of the agents using the track ROS service
        Arguments:
            image (Image): Top camera image
            agents_loc (List): List of agents location (x, y)
            track_start_loc (tuple): Start (x, y) location track drawn on the main camera image
        Returns:
            Image: Edited image with the circle
        """
        # For objects like box or bots
        for model_img, object_loc in zip(self.model_imgs[ModelImgNames.OBJECT_IMGS.value], objects_loc):
            loc_x, loc_y = self.get_model_imgs_on_track_loc(object_loc, track_start_loc)
            image = utils.plot_rectangular_image_on_main_image(image, model_img, (loc_x, loc_y))

        # For agents on the track
        for i, agent_img in enumerate(self.model_imgs[ModelImgNames.AGENT_IMGS.value]):
            # For plotting the agents
            agent_loc = agents_loc[i]
            loc_x, loc_y = self.get_model_imgs_on_track_loc(agent_loc, track_start_loc)
            image = utils.plot_rectangular_image_on_main_image(image, agent_img, (loc_x, loc_y))

            # For F1 race plot the number on top of the agents
            if self.is_f1_race_type:
                agent_num_img = self.model_imgs[ModelImgNames.AGENT_NUM_IMGS.value][i]
                loc_x = loc_x + XYPixelLoc.F1_AGENT_NUM_OFFSET.value[0]
                loc_y = loc_y - XYPixelLoc.F1_AGENT_NUM_OFFSET.value[1]
                image = utils.plot_rectangular_image_on_main_image(image, agent_num_img, (loc_x, loc_y))
        return image
