#!/usr/bin/env python3
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

"""
Fader class is used for smooth transition from one phase to another by gradually dimming the camera frames.
This class supports fading in and fading out. The gradient for fading in and fading out are pre-computed
and only the new frames is multiplied with these pre-computed stacks to provide the feature of
fading.
"""
from mp4_saving import utils


class Fader(object):
    """ Provide a smooth transition of the phases by fading in and out
    """
    def __init__(self, final_fading_image, fading_min_percent=0, fading_max_percent=1, num_frames=30):
        """  Fader class is used for smooth transition from one phase to another by gradually dimming the camera frames.

        This class supports fading in and fading out. The gradient for fading in and fading out are pre-computed
        and only the new frames is multiplied with these pre-computed stacks to provide the feature of
        fading.

        Args:
            final_fading_image (float): The gradient image that has to be applied at the end of fade out phase
            fading_min_percent (float): Starting multiplier to gradient alpha value. (default: {0})
            fading_max_percent (float): Ending multiplier to gradient alpha value.  (default: {1})
            num_frames (int): Number of frames the fading transitions occur.30 Frames = 2seconds if 15FPS(default: {30})
        """
        self._final_fading_image = final_fading_image
        self._fading_min_percent = max(0, fading_min_percent)
        self._fading_max_percent = min(1, fading_max_percent)
        self._num_frames = num_frames
        self._gradient_alpha_rgb_mul_list = list()
        self._one_minus_gradient_alpha_list = list()
        self._precomputed_gradients()
        # Since reset agent called twice. The pointer goes back to zero again and the frames
        # becomes brighter because this pointer goes back to zero.
        # Hence from the begining of wait state start with darker gradient
        self._pointer = len(self._gradient_alpha_rgb_mul_list) - 1

    def _precomputed_gradients(self):
        """ To optimize the fading behaviour. All the gradient calculation is pre-computed and are
        put into a list. The new frames are multiplied with these matrices and giving a fading functionality
        """
        fade_percent_interval = (self._fading_max_percent - self._fading_min_percent) / self._num_frames
        start = self._fading_min_percent
        while start < self._fading_max_percent:
            gradient_alpha_rgb_mul, one_minus_gradient_alpha = utils.get_gradient_values(self._final_fading_image,
                                                                                         multiplier=start)
            self._gradient_alpha_rgb_mul_list.append(gradient_alpha_rgb_mul)
            self._one_minus_gradient_alpha_list.append(one_minus_gradient_alpha)
            start += fade_percent_interval
        gradient_alpha_rgb_mul, one_minus_gradient_alpha = utils.get_gradient_values(
            self._final_fading_image, multiplier=self._fading_max_percent)
        self._gradient_alpha_rgb_mul_list.append(gradient_alpha_rgb_mul)
        self._one_minus_gradient_alpha_list.append(one_minus_gradient_alpha)

    def fade_in(self, background_image):
        """ The function takes the new frame from the camera and multiplies with the pre-computed gradients.
        This will allow it fade the image. (Brighter to darker image). The pointer keeps track of the which
        gradient to apply at a specific period of time.

        Args:
            background_image (Image): Frame from the camera

        Returns:
            (Image): The edited image where the gradient is applied.
        """
        image = utils.apply_gradient(background_image, self._gradient_alpha_rgb_mul_list[self._pointer],
                                     self._one_minus_gradient_alpha_list[self._pointer])
        self._pointer = min(self._pointer + 1, len(self._gradient_alpha_rgb_mul_list) - 1)
        return image

    def fade_out(self, background_image):
        """ The function takes the new frame from the camera and multiplies with the pre-computed gradients.
        This will allow it fade out. (Darker to Brighter image). The pointer keeps track of the which
        gradient to apply at a specific period of time. This iterates in the reverse order to that of fade_in

        Args:
            background_image (Image): Frame from the camera

        Returns:
            (Image): The edited image where the gradient is applied.
        """
        if self._pointer > 0:
            image = utils.apply_gradient(background_image,
                                         self._gradient_alpha_rgb_mul_list[self._pointer],
                                         self._one_minus_gradient_alpha_list[self._pointer])
            self._pointer = max(0, self._pointer - 1)
            return image
        return background_image
