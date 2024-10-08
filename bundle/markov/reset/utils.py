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

'''Utility methods for the reset’s module'''

from markov.agent_ctrl.constants import ConfigParams
from markov.reset.reset_rules_manager import ResetRulesManager
from markov.reset.rules.crash_reset_rule import CrashResetRule
from markov.reset.rules.immobilized_reset_rule import ImmobilizedResetRule
from markov.reset.rules.episode_complete_reset_rule import EpisodeCompleteResetRule
from markov.reset.rules.off_track_reset_rule import OffTrackResetRule
from markov.reset.rules.reverse_reset_rule import ReverseResetRule
from markov.reset.rules.race_time_rule import RaceTimeRule
from markov.virtual_event.constants import DEFAULT_RACE_DURATION


def construct_reset_rules_manager(config_dict):
    '''construct the reset reset rule manager

    Args:
        config_dict (dict): configuration dictionary

    Returns:
        ResetRulesManager: reset rules manager class instance
    '''
    reset_rules_manager = ResetRulesManager()
    reset_rules_manager.add(EpisodeCompleteResetRule(
        config_dict[ConfigParams.IS_CONTINUOUS.value],
        config_dict[ConfigParams.NUMBER_OF_TRIALS.value]))
    reset_rules_manager.add(ImmobilizedResetRule())
    reset_rules_manager.add(OffTrackResetRule())
    reset_rules_manager.add(CrashResetRule(config_dict[ConfigParams.AGENT_NAME.value]))
    reset_rules_manager.add(ReverseResetRule())
    if config_dict.get(ConfigParams.IS_VIRTUAL_EVENT.value, False):
        reset_rules_manager.add(RaceTimeRule(config_dict.get(
                                ConfigParams.RACE_DURATION.value, DEFAULT_RACE_DURATION)))
    return reset_rules_manager
