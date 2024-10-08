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
                                  XYPixelLoc,
                                  VirtualEventMP4Params)

LOG = Logger(__name__, logging.INFO).get_logger()


class VirtualEventWaitState(AbsFSMState):
    """Virtual Event Wait state
    """
    def __init__(self):
        """initialize Wait state with a finish time for how long to display check flag
        """
        LOG.info("[virtual event]: video edit state at {}".format(self))
        self._finish_time = time.time()

    def _execute(self, input_val):
        """Virtual Event state machine on event call

        Args:
            input_val (dict): input value dictionary

        Returns:
            self or VirtualEventPrepareState: self or next state that will transit to based on event
        """
        event, info_dict = input_val['event'], input_val['info_dict']
        major_cv_image = info_dict[VirtualEventMP4Params.MAJOR_CV_IMAGE.value]
        fader_obj = info_dict[VirtualEventMP4Params.FADER_OBJ.value]
        major_cv_image = fader_obj.fade_in(major_cv_image)
        if event == EpisodeStatus.PREPARE.value:
            # import in method to prevent circulr dependecy
            from mp4_saving.states.virtual_event_prepare_state import VirtualEventPrepareState
            # transit to PREPARE state
            return VirtualEventPrepareState(), info_dict
        # TODO: display wait icon
        info_dict['display_name'] = WAIT_DISPLAY_NAME
        info_dict['current_lap'] = WAIT_CURRENT_LAP
        info_dict['total_eval_milli_seconds'] = WAIT_TOTAL_EVAL_SECONDS
        info_dict['reset_counter'] = WAIT_RESET_COUNTER
        info_dict['speed'] = WAIT_SPEED
        info_dict[VirtualEventMP4Params.MAJOR_CV_IMAGE.value] = major_cv_image
        return self, info_dict
