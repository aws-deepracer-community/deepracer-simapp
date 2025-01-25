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

'''This module implements concrete reset rule for race time up'''
import logging
from markov.log_handler.logger import Logger
from markov.reset.abstract_reset_rule import AbstractResetRule
from markov.reset.constants import RaceCtrlStatus
from markov.track_geom.track_data import TrackData
from markov.metrics.constants import EpisodeStatus

LOG = Logger(__name__, logging.INFO).get_logger()

class RaceTimeRule(AbstractResetRule):
    name = EpisodeStatus.TIME_UP.value

    def __init__(self, race_duration):
        super(RaceTimeRule, self).__init__(RaceTimeRule.name)
        self._race_duration = race_duration

    def _update(self, agent_status):
        '''Update the race time up flag

        Args:
            agent_status (dict): agent status dictionary
        '''
        start_time = agent_status[RaceCtrlStatus.RACE_START_TIME.value]
        current_time = agent_status[RaceCtrlStatus.RACE_CURR_TIME.value]
        if (current_time - start_time) > self._race_duration:
            LOG.info("issue done. start_time: %s, current_time: %s",
                     start_time, current_time)
            self._done = True
