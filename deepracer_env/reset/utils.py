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

from deepracer_env.agent_ctrl.constants import ConfigParams
from deepracer_env.reset.reset_rules_manager import ResetRulesManager
from deepracer_env.reset.rules.crash_reset_rule import CrashResetRule
from deepracer_env.reset.rules.immobilized_reset_rule import ImmobilizedResetRule
from deepracer_env.reset.rules.episode_complete_reset_rule import EpisodeCompleteResetRule
from deepracer_env.reset.rules.off_track_reset_rule import OffTrackResetRule
from deepracer_env.reset.rules.reverse_reset_rule import ReverseResetRule


def construct_reset_rules_manager(config_dict):
    '''construct the reset rule manager

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
    return reset_rules_manager
