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

'''This module implements concrete reset rule for going reversed direction'''
import rospy

from markov.reset.abstract_reset_rule import AbstractResetRule
from markov.reset.constants import AgentCtrlStatus, AgentPhase
from markov.metrics.constants import EpisodeStatus


class ReverseResetRule(AbstractResetRule):
    name = EpisodeStatus.REVERSED.value

    def __init__(self):
        super(ReverseResetRule, self).__init__(ReverseResetRule.name)
        self._number_of_reverse_counts = int(rospy.get_param("NUMBER_OF_REVERSE_COUNTS", 15))
        self._reverse_count = 0

    def _update(self, agent_status):
        '''Update the reset rule done flag

        Args:
            agent_status (dict): agent status dictionary
        '''
        agent_phase = agent_status[AgentCtrlStatus.AGENT_PHASE.value]
        current_progress = agent_status[AgentCtrlStatus.CURRENT_PROGRESS.value]
        prev_progress = agent_status[AgentCtrlStatus.PREV_PROGRESS.value]

        if agent_phase == AgentPhase.RUN.value and current_progress < prev_progress:
            self._reverse_count += 1
        else:
            self._reverse_count = 0
        if self._reverse_count >= self._number_of_reverse_counts:
            self._reverse_count = 0
            self._done = True
