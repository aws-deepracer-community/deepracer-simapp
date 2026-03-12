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
from enum import Enum

from markov.rclpy_wrappers import ServiceProxyWrapper
from markov.domain_randomizations.abs_randomizer import AbstractRandomizer
from markov.log_handler.logger import Logger
from markov.domain_randomizations.constants import (GazeboServiceName,
                                                    RangeType,
                                                    RANGE_MIN, RANGE_MAX,
                                                    Color, Attenuation)

from std_msgs.msg import ColorRGBA
from deepracer_msgs.srv import SetLightProperties


logger = Logger(__name__, logging.INFO).get_logger()


class LightRandomizer(AbstractRandomizer):
    """Light Randomizer class"""
    def __init__(self, light_name, color_range=None, attenuation_range=None):
        """
        Constructor

        Args:
            light_name (str): name of the light
            color_range (dict): min-max of each color component (r, g, b).
                Valid format: {'r': {'min': 0.0, 'max': 1.0},
                               'g': {'min': 0.0, 'max': 1.0},
                               'b': {'min': 0.0, 'max': 1.0}}
            attenuation_range (dict): min-max of each attenuation component (constant, linear, quadratic).
                Valid format: {'constant': {'min': 0.0, 'max':1.0},
                               'linear': {'min': 0.0, 'max':1.0},
                               'quadratic': {'min': 0.0, 'max':1.0}}
        """
        super(LightRandomizer, self).__init__()
        self.light_name = light_name

        self.range = {RangeType.COLOR: {Color.R.value: {RANGE_MIN: 0.0, RANGE_MAX: 1.0},
                                        Color.G.value: {RANGE_MIN: 0.0, RANGE_MAX: 1.0},
                                        Color.B.value: {RANGE_MIN: 0.0, RANGE_MAX: 1.0}},
                      RangeType.ATTENUATION: {Attenuation.CONSTANT.value: {RANGE_MIN: 0.0, RANGE_MAX: 1.0},
                                              Attenuation.LINEAR.value: {RANGE_MIN: 0.0, RANGE_MAX: 1.0},
                                              Attenuation.QUADRATIC.value: {RANGE_MIN: 0.0, RANGE_MAX: 1.0}}}
        if color_range:
            self.range[RangeType.COLOR].update(color_range)
        if attenuation_range:
            self.range[RangeType.ATTENUATION].update(attenuation_range)

        # ROS Services
        self.set_light_prop = ServiceProxyWrapper(GazeboServiceName.SET_LIGHT_PROPERTIES.value, SetLightProperties)
        
        logger.info("LightRandomizer initialized for light: %s", self.light_name)

    def _randomize(self):
        req = SetLightProperties.Request()
        req.light_name = self.light_name

        color_range = self.range[RangeType.COLOR]
        req.diffuse = ColorRGBA(r=np.random.uniform(color_range[Color.R.value][RANGE_MIN],
                                                     color_range[Color.R.value][RANGE_MAX]),
                                g=np.random.uniform(color_range[Color.G.value][RANGE_MIN],
                                                     color_range[Color.G.value][RANGE_MAX]),
                                b=np.random.uniform(color_range[Color.B.value][RANGE_MIN],
                                                     color_range[Color.B.value][RANGE_MAX]),
                                a=1.0)
        
        # Set specular to match diffuse for consistent lighting
        req.specular = ColorRGBA(r=req.diffuse.r * 0.5, g=req.diffuse.g * 0.5,
                                 b=req.diffuse.b * 0.5, a=1.0)

        attenuation_range = self.range[RangeType.ATTENUATION]
        req.attenuation_constant = np.random.uniform(attenuation_range[Attenuation.CONSTANT.value][RANGE_MIN],
                                                     attenuation_range[Attenuation.CONSTANT.value][RANGE_MAX])
        req.attenuation_linear = np.random.uniform(attenuation_range[Attenuation.LINEAR.value][RANGE_MIN],
                                                   attenuation_range[Attenuation.LINEAR.value][RANGE_MAX])
        req.attenuation_quadratic = np.random.uniform(attenuation_range[Attenuation.QUADRATIC.value][RANGE_MIN],
                                                      attenuation_range[Attenuation.QUADRATIC.value][RANGE_MAX])
        
        res = self.set_light_prop(req)
        
        if res is None:
            logger.warning("SetLightProperties for '%s' returned None", self.light_name)
        elif not res.success:
            logger.warning("SetLightProperties for '%s' failed: %s", self.light_name, res.status_message)
        else:
            logger.debug("Light '%s' randomized: diffuse=(%.2f,%.2f,%.2f)", 
                        self.light_name, req.diffuse.r, req.diffuse.g, req.diffuse.b)
