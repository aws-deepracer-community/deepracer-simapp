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

'''This module implements concrete reset rule for episode completion'''

from markov.reset.abstract_reset_rule import AbstractResetRule
from markov.reset.constants import AgentCtrlStatus, AgentInfo
from markov.track_geom.track_data import TrackData
from markov.metrics.constants import EpisodeStatus

class EpisodeCompleteResetRule(AbstractResetRule):
    name = EpisodeStatus.EPISODE_COMPLETE.value

    def __init__(self, is_continuous, number_of_trials):
        super(EpisodeCompleteResetRule, self).__init__(EpisodeCompleteResetRule.name)
        self._is_continuous = is_continuous
        self._number_of_trials = number_of_trials
        self._lap_count = 0

    def _update(self, agent_status):
        '''Update the lap complete rule done flag

        Args:
            agent_status (dict): agent status dictionary

        Returns:
            dict: dictionary contains the agent lap complete info
        '''
        current_progress = agent_status[AgentCtrlStatus.CURRENT_PROGRESS.value]
        is_lap_complete = current_progress >= 100.0
        if is_lap_complete:
            self._lap_count += 1
        if self._is_continuous:
            self._done = (self._lap_count >= self._number_of_trials)
        else:
            self._done = is_lap_complete
        return {AgentInfo.LAP_COUNT.value: self._lap_count,
                AgentInfo.CURRENT_PROGRESS.value: current_progress}
