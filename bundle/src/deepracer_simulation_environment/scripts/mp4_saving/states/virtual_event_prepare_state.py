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

"""this module implements all virtual event state machine states"""

import time
import logging
import cv2

from markov.log_handler.logger import Logger
from markov.metrics.constants import EpisodeStatus
from markov.state_machine.abs_fsm_state import AbsFSMState
from markov.virtual_event.constants import (PAUSE_TIME_BEFORE_START,
                                            WAIT_TOTAL_EVAL_SECONDS,
                                            WAIT_SPEED,
                                            PAUSE_TIME_AFTER_FINISH,
                                            WAIT_DISPLAY_NAME,
                                            WAIT_CURRENT_LAP,
                                            WAIT_RESET_COUNTER)
from mp4_saving import utils
from mp4_saving.constants import (IconographicImageSize,
                                  TrackAssetsIconographicPngs, RACE_COMPLETE_Y_OFFSET,
                                  Mp4Parameter, VIRTUAL_EVENT_PREPARE_DIGIT_FONT,
                                  RaceCarColorToRGB, VirtualEventMP4Params,
                                  VirtualEventXYPixelLoc,
                                  VirtualEventIconographicPngs)

LOG = Logger(__name__, logging.INFO).get_logger()


class VirtualEventPrepareState(AbsFSMState):
    """Virtual Event Prepare State

    In the Prepare state, racecar will count down 3, 2, ... 0
    """
    def __init__(self):
        """initialize Prepare state with digit to display
        """
        LOG.info("[virtual event]: video edit state at {}".format(self))
        self._digit = int(PAUSE_TIME_BEFORE_START)
        self._amazon_ember_heavy_100px = utils.get_font('AmazonEmber-Heavy', 100)
        frame_x, frame_y = Mp4Parameter.FRAME_SIZE.value
        self._loc_x, self._loc_y = (frame_x - VIRTUAL_EVENT_PREPARE_DIGIT_FONT // 2) // 2, \
            (frame_y - VIRTUAL_EVENT_PREPARE_DIGIT_FONT) // 2
        self._icon_image = utils.get_image(VirtualEventIconographicPngs.SET.value)
        self._icon_image = cv2.cvtColor(self._icon_image, cv2.COLOR_RGBA2BGRA)

    def _execute(self, input_val):
        """Virtual Event state machine on event call

        Args:
            input_val (dict): input value dictionary

        Returns:
            self or VirtualEventRunState: self or next state that will transit to based on event
        """
        event, info_dict = input_val['event'], input_val['info_dict']
        major_cv_image = info_dict[VirtualEventMP4Params.MAJOR_CV_IMAGE.value]

        # During the prepare phase, for smooth transition we would like to fade out the camera image
        # (Darker to Brighter image).
        fader_obj = info_dict[VirtualEventMP4Params.FADER_OBJ.value]
        major_cv_image = fader_obj.fade_out(major_cv_image)

        if event == EpisodeStatus.PREPARE.value:
            # get params from info_dict
            countdown_timer = info_dict['countdown_timer']
            # display countdown digits
            if 0 < countdown_timer <= self._digit - 1:
                self._digit -= 1
            # write SET icon
            icon_x, icon_y = VirtualEventXYPixelLoc.ICON.value
            major_cv_image = utils.plot_rectangular_image_on_main_image(
                major_cv_image, self._icon_image, (icon_x, icon_y))
            # write count down digit
            countdown_digit = "{}".format(self._digit)
            major_cv_image = utils.write_text_on_image(image=major_cv_image, text=countdown_digit,
                                                       loc=(self._loc_x, self._loc_y),
                                                       font=self._amazon_ember_heavy_100px,
                                                       font_color=RaceCarColorToRGB.White.value,
                                                       font_shadow_color=RaceCarColorToRGB.Black.value)
            # update info dict
            info_dict[VirtualEventMP4Params.MAJOR_CV_IMAGE.value] = major_cv_image
            info_dict[VirtualEventMP4Params.TOTAL_EVAL_SECONDS.value] = WAIT_TOTAL_EVAL_SECONDS
            info_dict[VirtualEventMP4Params.SPEED.value] = WAIT_SPEED
            # stay at PREPARE state
            return self, info_dict
        # import in method to prevent circualr dependecy
        from mp4_saving.states.virtual_event_run_state import VirtualEventRunState
        # transit to RUN state
        return VirtualEventRunState(current_sector=0), info_dict
