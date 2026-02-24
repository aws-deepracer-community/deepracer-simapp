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

import os
import json
import logging
import rospy
import cv2

from threading import Thread
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                          SIMAPP_EVENT_ERROR_CODE_500)
from markov.metrics.constants import EpisodeStatus
from markov.state_machine.abs_fsm_state import AbsFSMState
from markov.virtual_event.constants import DEFAULT_RACE_DURATION
from markov.boto.s3.files.virtual_event_best_sector_time import VirtualEventBestSectorTime
from markov.boto.s3.utils import get_s3_key
from markov.boto.s3.constants import (SECTOR_TIME_LOCAL_PATH,
                                      SECTOR_TIME_S3_POSTFIX,
                                      TrackSectorTime,
                                      SECTOR_X_FORMAT,
                                      SECTOR_TIME_FORMAT_DICT)
from markov.utils import get_s3_kms_extra_args
from mp4_saving import utils
from mp4_saving.constants import (VirtualEventMP4Params,
                                  VirtualEventXYPixelLoc,
                                  VirtualEventIconographicPngs)

LOG = Logger(__name__, logging.INFO).get_logger()


class VirtualEventRunState(AbsFSMState):
    """Virtual Event Run State

    In the Run state, racecar will run as normal
    """
    def __init__(self, current_sector=0):
        """initialize Run state and log

        Args:
            current_sector (int): current sector index 0-based.
        """
        self._total_sectors = int(rospy.get_param("NUM_SECTORS", "3"))
        if self._total_sectors == 0:
            log_and_exit("[virtual event]: Virtual event run state with 0 total sectors. \
                         This needs to be at least 1",
                         SIMAPP_SIMULATION_WORKER_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)

        # current sector index 0 is sector1 and so on so forth
        self._current_sector = current_sector % self._total_sectors
        self._target_progress = (100.00 / self._total_sectors) * (self._current_sector + 1)
        self._total_laps = int(rospy.get_param("NUMBER_OF_TRIALS", 3))
        self._race_duration = int(rospy.get_param("RACE_DURATION", DEFAULT_RACE_DURATION)) * 1000

        # VirtualEventBestSectorTime S3 upload instance
        # use the s3 bucket and prefix for yaml file stored as environment variable because
        # here is SimApp use only. For virtual event there is no s3 bucket and prefix past
        # through yaml file. All are past through sqs. For simplicity, reuse the yaml s3 bucket
        # and prefix environment variable.
        self._virtual_event_best_sector_time = VirtualEventBestSectorTime(
            bucket=os.environ.get("YAML_S3_BUCKET", ''),
            s3_key=get_s3_key(os.environ.get("YAML_S3_PREFIX", ''), SECTOR_TIME_S3_POSTFIX),
            region_name=os.environ.get("APP_REGION", "us-east-1"),
            local_path=SECTOR_TIME_LOCAL_PATH)

        # Go icon image
        self._icon_image = utils.get_image(VirtualEventIconographicPngs.GO.value)
        self._icon_image = cv2.cvtColor(self._icon_image, cv2.COLOR_RGBA2BGRA)

        # init number of sectors to plot
        # for 3 sectors example
        # if racer is at sector 1 (idx 0): plot sector 1, 2, and 3
        # if racer is at sector 2 (idx 1): plot sector 1
        # if racer is at sector 3 (idx 2): plot sector 1, and 2
        num_sectors_to_plot = self._current_sector
        if self._current_sector == 0:
            num_sectors_to_plot = self._total_sectors

        # init number of sectors to plot
        self._sectors = [SECTOR_X_FORMAT.format(idx + 1) for idx in range(num_sectors_to_plot)]

        # sector format string for best session, best personal, and current personal
        self._best_session_format = SECTOR_TIME_FORMAT_DICT[TrackSectorTime.BEST_SESSION]
        self._best_personal_format = SECTOR_TIME_FORMAT_DICT[TrackSectorTime.BEST_PERSONAL]
        self._current_personal_format = SECTOR_TIME_FORMAT_DICT[TrackSectorTime.CURRENT_PERSONAL]

        LOG.info("[virtual event]: video edit state at {} for sector {} with target progress {}\
            ".format(self, self._current_sector + 1, self._target_progress))

    def _execute(self, input_val):
        """Virtual Event state machine on event call

        Args:
            input_val (dict): input value dictionary

        Returns:
            self or VirtualEventFinishState or VirtualEventRunSector1State:
            self or next state that will transit to based on event
        """
        event, info_dict = input_val['event'], input_val['info_dict']
        curr_progress = info_dict[VirtualEventMP4Params.CURR_PROGRESS.value]
        curr_eval_time = info_dict[VirtualEventMP4Params.TOTAL_EVAL_SECONDS.value]
        last_eval_time = info_dict[VirtualEventMP4Params.LAST_EVAL_SECONDS.value]
        major_cv_image = info_dict[VirtualEventMP4Params.MAJOR_CV_IMAGE.value]

        # update current lap time
        # for example: if racer is currently in the middle of sector 2,
        # we need to add current personal sector 1's time into current lap time
        info_dict[VirtualEventMP4Params.CURR_LAP_TIME.value] = curr_eval_time - last_eval_time
        sector_time_dict = info_dict[VirtualEventMP4Params.SECTOR_TIMES.value]
        for idx in range(self._current_sector):
            sector = SECTOR_X_FORMAT.format(idx + 1)
            info_dict[VirtualEventMP4Params.CURR_LAP_TIME.value] += \
                sector_time_dict[self._current_personal_format.format(sector)]

        # overwrite the current lap time if more than total race duration for display
        if info_dict[VirtualEventMP4Params.CURR_LAP_TIME.value] >= self._race_duration:
            info_dict[VirtualEventMP4Params.CURR_LAP_TIME.value] = self._race_duration

        # write GO icon
        icon_x, icon_y = VirtualEventXYPixelLoc.ICON.value
        major_cv_image = utils.plot_rectangular_image_on_main_image(
            major_cv_image, self._icon_image, (icon_x, icon_y))

        # race time over event or episode complete: move to FINISH state
        if event == EpisodeStatus.TIME_UP.value:
            LOG.info("[virtual event]: video edit state {} -> Finish with time up\
                ".format(SECTOR_X_FORMAT.format(self._current_sector + 1)))
            from mp4_saving.states.virtual_event_finish_state import VirtualEventFinishState
            return VirtualEventFinishState(total_sectors=self._current_sector), info_dict
        # racer finish all laps: move to FINISH state
        elif info_dict[VirtualEventMP4Params.CURRENT_LAP.value] > self._total_laps:
            LOG.info("[virtual event]: video edit state {} -> Finish with lap completion\
                ".format(SECTOR_X_FORMAT.format(self._current_sector + 1)))
            from mp4_saving.states.virtual_event_finish_state import VirtualEventFinishState
            if (self._current_sector + 1) == self._total_sectors and \
                    (curr_progress == 100.00 or 0 <= curr_progress <= 100.00 / self._total_sectors):
                info_dict = self._update_sector_times(info_dict, self._current_sector)
                return VirtualEventFinishState(total_sectors=self._current_sector + 1), info_dict
            return VirtualEventFinishState(total_sectors=self._current_sector), info_dict

        #############################################################################################
        # need to handle lap complete 100 target progress specifically. Sometime, we have seen 100. #
        # progress ros message missing. Other time, we have seen 100 progress ros message occur     #
        # muliple times. Also need to handle transition from lap complete 100 to 0 carefully as well#
        # because it is also possible that when we transit to sector 1 while progress is still 100. #
        #############################################################################################
        # last sector state
        # handle progress 100 ros message comes or 100 ros message missing
        elif (self._current_sector + 1) == self._total_sectors and \
                (curr_progress == 100.00 or 0 <= curr_progress <= 100.00 / self._total_sectors):
            return self._on_exit_last_sector_state(
                info_dict=info_dict,
                curr_progress=curr_progress)

        # first sector state
        # handle progress 100 ros message comes multiple times
        # hanlde there is progress like 99.18 comes after progress 100
        elif (self._current_sector == 0) and \
                (curr_progress != 100.00 and 100.00 / self._total_sectors < curr_progress < 2 * 100.00 / self._total_sectors):
            return self._on_exit_first_sector_state(
                info_dict=info_dict,
                curr_progress=curr_progress)

        # other sectors
        # handle all other normal cases besides special case in the first and last sector
        elif ((self._current_sector + 1) != self._total_sectors) and (self._current_sector != 0) and\
                curr_progress > self._target_progress:
            return self._on_exit_current_sector_state(
                info_dict=info_dict,
                curr_progress=curr_progress)
        # current sector
        else:
            return self._execute_current_sector(
                major_cv_image=major_cv_image,
                info_dict=info_dict)

    def _on_exit_last_sector_state(self, info_dict, curr_progress):
        """handle transition out of last sector specifically

        Args:
            info_dict (dict): information dict for state execution
            curr_progress (float between 0 and 100.00): current progress

        Returns:
            tuple(AbsFSMState, dict): specific AbsFSMState class instance and info dict tuple
        """
        # update current personal, personal best, and sector best times
        info_dict = self._update_sector_times(info_dict, self._current_sector)

        LOG.info("[virtual event]: video edit state {} -> sector1 with progress {}\
            ".format(SECTOR_X_FORMAT.format(self._total_sectors),
                     curr_progress))

        # (self._current_sector + 1) % self._total_sectors) should always be index 0 here
        return VirtualEventRunState(
            current_sector=(self._current_sector + 1) % self._total_sectors), info_dict

    def _on_exit_first_sector_state(self, info_dict, curr_progress):
        """handle transition out of first sector specifically

        Args:
            info_dict (dict): information dict for state execution
            curr_progress (float between 0 and 100.00): current progress

        Returns:
            tuple(AbsFSMState, dict): specific AbsFSMState class instance and info dict tuple
        """
        # update current personal, personal best, and sector best times
        info_dict = self._update_sector_times(info_dict, self._current_sector)

        # clean up all current personal sector times besides sector 1 for display purpose
        sector_time_dict = info_dict[VirtualEventMP4Params.SECTOR_TIMES.value]
        for idx in range(1, self._total_sectors):
            sector = SECTOR_X_FORMAT.format(idx + 1)
            sector_time_dict[self._current_personal_format.format(sector)] = float("inf")
        info_dict[VirtualEventMP4Params.SECTOR_TIMES.value].update(sector_time_dict)

        start_sector = (self._current_sector + 1) % self._total_sectors
        end_sector = (self._current_sector + 2) % self._total_sectors
        LOG.info("[virtual event]: video edit state {} -> {} with progress {}\
            ".format(SECTOR_X_FORMAT.format(start_sector) if start_sector != 0 else SECTOR_X_FORMAT.format(self._total_sectors),
                     SECTOR_X_FORMAT.format(end_sector) if end_sector != 0 else SECTOR_X_FORMAT.format(self._total_sectors),
                     curr_progress))

        return VirtualEventRunState(
            current_sector=(self._current_sector + 1) % self._total_sectors), info_dict

    def _on_exit_current_sector_state(self, info_dict, curr_progress):
        """handle transition out of other current sectors generally

        Args:
            info_dict (dict): information dict for state execution
            curr_progress (float between 0 and 100.00): current progress

        Returns:
            tuple(AbsFSMState, dict): specific AbsFSMState class instance and info dict tuple
        """
        # update current personal, personal best, and sector best times
        info_dict = self._update_sector_times(info_dict, self._current_sector)

        start_sector = (self._current_sector + 1) % self._total_sectors
        end_sector = (self._current_sector + 2) % self._total_sectors
        LOG.info("[virtual event]: video edit state {} -> {} with progress {}\
            ".format(SECTOR_X_FORMAT.format(start_sector) if start_sector != 0 else SECTOR_X_FORMAT.format(self._total_sectors),
                     SECTOR_X_FORMAT.format(end_sector) if end_sector != 0 else SECTOR_X_FORMAT.format(self._total_sectors),
                     curr_progress))

        return VirtualEventRunState(
            current_sector=(self._current_sector + 1) % self._total_sectors), info_dict

    def _execute_current_sector(self, info_dict, major_cv_image):
        """handle current sector generally

        Args:
            info_dict (dict): information dict for state execution
            major_cv_image (cv2.img) cv2 image format

        Returns:
            tuple(self, dict): self and info dict tuple
        """
        # compare current personal sector with best personal and best session
        # Green for personal best, yellow for slower sectors, and purple for session best.
        # plot sector 1 and sector 2
        x_min, x_max, y_min, y_max = \
            info_dict[VirtualEventMP4Params.X_MIN.value], \
            info_dict[VirtualEventMP4Params.X_MAX.value], \
            info_dict[VirtualEventMP4Params.Y_MIN.value], \
            info_dict[VirtualEventMP4Params.Y_MAX.value]
        sectors_img_dict = info_dict[VirtualEventMP4Params.SECTOR_IMAGES.value]
        sector_time_dict = info_dict[VirtualEventMP4Params.SECTOR_TIMES.value]
        for sector in self._sectors:
            if sector_time_dict[self._current_personal_format.format(sector)] != float("inf") and \
                    sector_time_dict[self._best_session_format.format(sector)] is not None:
                sector_color = utils.get_sector_color(
                    best_session_time=sector_time_dict[self._best_session_format.format(sector)],
                    best_personal_time=sector_time_dict[self._best_personal_format.format(sector)],
                    current_personal_time=sector_time_dict[self._current_personal_format.format(sector)])
                major_cv_image = utils.overlay_sector_color_on_track(
                    major_cv_image=major_cv_image,
                    sector_img=sectors_img_dict[sector][sector_color],
                    x_min=x_min,
                    x_max=x_max,
                    y_min=y_min,
                    y_max=y_max)

        # update major cv image in info dict and return
        info_dict[VirtualEventMP4Params.MAJOR_CV_IMAGE.value] = major_cv_image
        return self, info_dict

    def _update_sector_times(self, info_dict, sector_idx):
        """update curent personal, best personal, and sector best times.
        If there is a sector best time, upload the sector best time into s3 in
        a separted thread

        Args:
            info_dict(dict): infomation dictionary contains all necesary info to update sector times
            sector_idx(int): sector index for sector1 index is 0 and so on so forth

        Returns:
            dict: updated info_dict
        """
        sector = SECTOR_X_FORMAT.format(sector_idx + 1)
        curr_eval_time = info_dict[VirtualEventMP4Params.TOTAL_EVAL_SECONDS.value]
        last_eval_time = info_dict[VirtualEventMP4Params.LAST_EVAL_SECONDS.value]

        # get sector_time_dict
        sector_time_dict = info_dict[VirtualEventMP4Params.SECTOR_TIMES.value]
        sector_time_dict[self._current_personal_format.format(sector)] \
            = curr_eval_time - last_eval_time
        info_dict[VirtualEventMP4Params.LAST_EVAL_SECONDS.value] = curr_eval_time

        if sector_time_dict[self._best_session_format.format(sector)] is not None:
            # update sector best personal time
            if sector_time_dict[self._current_personal_format.format(sector)] <= \
                    sector_time_dict[self._best_personal_format.format(sector)]:
                sector_time_dict[self._best_personal_format.format(sector)] = \
                    sector_time_dict[self._current_personal_format.format(sector)]

            # update sector best session time
            if sector_time_dict[self._current_personal_format.format(sector)] <= \
                    sector_time_dict[self._best_session_format.format(sector)]:
                sector_time_dict[self._best_session_format.format(sector)] = \
                    sector_time_dict[self._current_personal_format.format(sector)]
                # persist the updated sector best session time with
                # other sectors into s3 for robomaker crash backup
                # in a new thread
                Thread(target=self._virtual_event_best_sector_time.persist,
                       args=(json.dumps({SECTOR_X_FORMAT.format(idx + 1):
                                         sector_time_dict[self._best_session_format.format(
                                             SECTOR_X_FORMAT.format(idx + 1))]
                                         for idx in range(self._total_sectors)}),
                             get_s3_kms_extra_args())).start()

        # update sector_time_dict to the latest
        info_dict[VirtualEventMP4Params.SECTOR_TIMES.value].update(sector_time_dict)
        return info_dict
