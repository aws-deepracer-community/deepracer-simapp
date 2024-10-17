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

'''This module implements concrete reset rule for off track'''

from markov.reset.abstract_reset_rule import AbstractResetRule
from markov.reset.constants import AgentCtrlStatus
from markov.track_geom.track_data import TrackData
from markov.track_geom.constants import AgentPos
from markov.metrics.constants import EpisodeStatus

class OffTrackResetRule(AbstractResetRule):
    name = EpisodeStatus.OFF_TRACK.value

    def __init__(self):
        super(OffTrackResetRule, self).__init__(OffTrackResetRule.name)
        self._track_data = TrackData.get_instance()

    def _update(self, agent_status):
        '''Update the off track rule done flag

        Args:
            agent_status (dict): agent status dictionary
        '''
        pos_dict = agent_status[AgentCtrlStatus.POS_DICT.value]
        is_off_track = not any(self._track_data.points_on_track(pos_dict[AgentPos.LINK_POINTS.value]))
        self._done = is_off_track
