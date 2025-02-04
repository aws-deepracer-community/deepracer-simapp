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

import logging
import numpy as np

from markov.domain_randomizations.abs_randomizer import AbstractRandomizer
from markov.log_handler.logger import Logger
from markov.domain_randomizations.constants import RangeType, RANGE_MIN, RANGE_MAX, Color

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteLight
from geometry_msgs.msg import Pose, Point, Quaternion

logger = Logger(__name__, logging.INFO).get_logger()


class LightRandomizer(AbstractRandomizer):

    def __init__(self, light_name, color_range=None):

        super(LightRandomizer, self).__init__()
        self.light_name = light_name

        # Set default ranges
        self.range = {
            RangeType.COLOR: {
                Color.R.value: {RANGE_MIN: 0.0, RANGE_MAX: 1.0},
                Color.G.value: {RANGE_MIN: 0.0, RANGE_MAX: 1.0},
                Color.B.value: {RANGE_MIN: 0.0, RANGE_MAX: 1.0},
            }
        }

        if color_range:
            self.range[RangeType.COLOR].update(color_range)

        # ROS Service proxies
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        rospy.wait_for_service("/gazebo/delete_light")
        self.delete_light = rospy.ServiceProxy("/gazebo/delete_light", DeleteLight)

    def _delete_light(self):
        try:
            self.delete_light(self.light_name)
            logger.info(f"Deleted light: {self.light_name}")
        except rospy.ServiceException as ex:
            logger.warning(f"Failed to delete light {self.light_name}: {str(ex)}")

    def _generate_light_sdf(self):

        # Generate the standalone light SDF
        if self.light_name == "Light 1":

            # Randomize color
            color_range = self.range[RangeType.COLOR]
            r = np.random.uniform(color_range[Color.R.value][RANGE_MIN], color_range[Color.R.value][RANGE_MAX])
            g = np.random.uniform(color_range[Color.G.value][RANGE_MIN], color_range[Color.G.value][RANGE_MAX])
            b = np.random.uniform(color_range[Color.B.value][RANGE_MIN], color_range[Color.B.value][RANGE_MAX])

            # Randomize position
            x = np.random.uniform(-5.0, 5.0)
            y = np.random.uniform(-5.0, 5.0)
            z = np.random.uniform(2.0, 5.0)

            return f"""<sdf version="1.6">
  <light name="{self.light_name}" type="point">
    <pose>{x} {y} {z} 0 0 0</pose>
    <diffuse>{r} {g} {b} 1</diffuse>
    <specular>.1 .1 .1 1</specular>
    <attenuation>
        <constant>0.4</constant>
        <linear>0.01</linear>
        <quadratic>0.00</quadratic>
        <range>50</range>
    </attenuation>
    <direction>0 0 -1</direction>
    <cast_shadows>false</cast_shadows>
  </light>
</sdf>"""
        elif self.light_name == "sun":

            # Randomize attenuation values
            constant = np.random.uniform(0.0, 1.0)
            linear = np.random.uniform(0.0, 1.0)
            quadratic = np.random.uniform(0.0, 1.0)

            # Slightly randomize brightness
            brightness = np.random.uniform(0.5, 1)
            r, g, b = brightness, brightness, brightness
            return f"""<sdf version="1.6">
  <light name="{self.light_name}" type="directional">
    <pose>0.0 0.0 15.0 0 0 0</pose>
    <diffuse>{r} {g} {b} 1</diffuse>
    <cast_shadows>false</cast_shadows>
    <attenuation>
      <constant>{constant}</constant>
      <linear>{linear}</linear>
      <quadratic>{quadratic}</quadratic>
      <range>100</range>
    </attenuation>
  </light>
</sdf>"""

    def _spawn_light(self, sdf_content):
        initial_pose = Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))  # Placeholder pose
        try:
            res = self.spawn_model(self.light_name, sdf_content, "", initial_pose, "world")
            if not res.success:
                logger.error(f"Failed to spawn light {self.light_name}: {res.status_message}")
            else:
                logger.info(f"Spawned new light: {self.light_name}")
        except rospy.ServiceException as ex:
            logger.error(f"Service call to spawn light failed: {str(ex)}")

    def _randomize(self):
        if self.light_name not in ["sun", "Light 1"]:
            logger.info(f"Skipping randomization for light: {self.light_name}")
            return  # Skip randomization for lights other than "sun" and "Light 1"

        # Delete the existing light
        self._delete_light()

        # Generate the new light SDF
        sdf_content = self._generate_light_sdf()

        # Spawn the new light
        self._spawn_light(sdf_content)
