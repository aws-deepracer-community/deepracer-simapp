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
import rospy
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
from markov.boto.s3.constants import (TrackSectorTime,
                                      SECTOR_X_FORMAT,
                                      SECTOR_TIME_FORMAT_DICT)
from mp4_saving import utils
from mp4_saving.constants import (IconographicImageSize,
                                  TrackAssetsIconographicPngs, RACE_COMPLETE_Y_OFFSET,
                                  XYPixelLoc,
                                  VirtualEventMP4Params,
                                  VirtualEventXYPixelLoc,
                                  VirtualEventIconographicPngs,
                                  SECTOR_COLORS_DICT,
                                  TrackSectorColors,
                                  TrackColors)

LOG = Logger(__name__, logging.INFO).get_logger()


class VirtualEventFinishState(AbsFSMState):
    """Virtual Event Finish state

    In Finish state, display check flag for several seconds
    """
    def __init__(self, total_sectors):
        """initialize Finish state with a finish time for how long to display check flag

        Args:
            total_sectors (int): total number of sectors
        """
        LOG.info("[virtual event]: video edit state at {}".format(self))
        self._icon_image = utils.get_image(VirtualEventIconographicPngs.FINISH.value)
        self._icon_image = cv2.cvtColor(self._icon_image, cv2.COLOR_RGBA2BGRA)
        self._sectors = [SECTOR_X_FORMAT.format(idx + 1) for idx in range(total_sectors)]

    def _execute(self, input_val):
        """Virtual Event state machine on event call

        Args:
            input_val (dict): input value dictionary

        Returns:
            self or VirtualEventPrepareState: self or next state that will transit to based on event
        """
        event, info_dict = input_val['event'], input_val['info_dict']
        current_progress = info_dict[VirtualEventMP4Params.CURR_PROGRESS.value]
        current_lap = info_dict[VirtualEventMP4Params.CURRENT_LAP.value]
        x_min, x_max, y_min, y_max = \
            info_dict[VirtualEventMP4Params.X_MIN.value], \
            info_dict[VirtualEventMP4Params.X_MAX.value], \
            info_dict[VirtualEventMP4Params.Y_MIN.value], \
            info_dict[VirtualEventMP4Params.Y_MAX.value]
        major_cv_image = info_dict[VirtualEventMP4Params.MAJOR_CV_IMAGE.value]

        # overwrite the finish speed to 0
        info_dict[VirtualEventMP4Params.SPEED.value] = WAIT_SPEED

        # write FINISH
        icon_x, icon_y = VirtualEventXYPixelLoc.ICON.value
        major_cv_image = utils.plot_rectangular_image_on_main_image(
            major_cv_image, self._icon_image, (icon_x, icon_y))

        # compare current personal sector with best personal and best session
        # Green for personal best, yellow for slower sectors, and purple for session best.
        # plot sector 1, sector 2, and sector 3
        sectors_img_dict = info_dict[VirtualEventMP4Params.SECTOR_IMAGES.value]
        sector_time_dict = info_dict[VirtualEventMP4Params.SECTOR_TIMES.value]
        for sector in self._sectors:
            if sector_time_dict[SECTOR_TIME_FORMAT_DICT[TrackSectorTime.CURRENT_PERSONAL].format(sector)] != float("inf") and \
                    sector_time_dict[SECTOR_TIME_FORMAT_DICT[TrackSectorTime.BEST_SESSION].format(sector)] is not None:
                sector_color = utils.get_sector_color(
                    best_session_time=sector_time_dict[SECTOR_TIME_FORMAT_DICT[TrackSectorTime.BEST_SESSION].format(sector)],
                    best_personal_time=sector_time_dict[SECTOR_TIME_FORMAT_DICT[TrackSectorTime.BEST_PERSONAL].format(sector)],
                    current_personal_time=sector_time_dict[SECTOR_TIME_FORMAT_DICT[TrackSectorTime.CURRENT_PERSONAL].format(sector)])
                major_cv_image = utils.overlay_sector_color_on_track(
                    major_cv_image=major_cv_image,
                    sector_img=sectors_img_dict[sector][sector_color],
                    x_min=x_min,
                    x_max=x_max,
                    y_min=y_min,
                    y_max=y_max)

        # During the finish phase, for smooth transition we would like to fade the camera image (Brighter to Darker image).
        fader_obj = info_dict[VirtualEventMP4Params.FADER_OBJ.value]
        major_cv_image = fader_obj.fade_in(major_cv_image)

        # update info dict
        info_dict['major_cv_image'] = major_cv_image
        if event == EpisodeStatus.PREPARE.value:
            # import in method to prevent circular dependency
            from mp4_saving.states.virtual_event_prepare_state import VirtualEventPrepareState
            # transit to PREPARE state
            return VirtualEventPrepareState(), info_dict
        # stay at FINISH state
        return self, info_dict
