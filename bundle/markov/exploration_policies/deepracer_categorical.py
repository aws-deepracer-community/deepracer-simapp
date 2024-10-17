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

from typing import List

import numpy as np

from rl_coach.core_types import RunPhase, ActionType
from rl_coach.exploration_policies.exploration_policy import DiscreteActionExplorationPolicy, ExplorationParameters
from rl_coach.spaces import ActionSpace


class DeepRacerCategoricalParameters(ExplorationParameters):
    def __init__(self, use_stochastic_evaluation_policy=False):
        super().__init__()
        self.use_stochastic_evaluation_policy = use_stochastic_evaluation_policy

    @property
    def path(self):
        return 'markov.exploration_policies.deepracer_categorical:DeepRacerCategorical'


class DeepRacerCategorical(DiscreteActionExplorationPolicy):
    """
    Categorical exploration policy is intended for discrete action spaces. It expects the action values to
    represent a probability distribution over the action, from which a single action will be sampled.
    In evaluation, the action that has the highest probability will be selected. This is particularly useful for
    actor-critic schemes, where the actors output is a probability distribution over the actions.
    """
    def __init__(self, action_space: ActionSpace, use_stochastic_evaluation_policy: bool = False):
        """
        :param action_space: the action space used by the environment
        """
        super().__init__(action_space)
        self.use_stochastic_evaluation_policy = use_stochastic_evaluation_policy

    def get_action(self, action_values: List[ActionType]) -> (ActionType, List[float]):
        if self.phase == RunPhase.TRAIN or self.use_stochastic_evaluation_policy:
            # choose actions according to the probabilities
            action = np.random.choice(self.action_space.actions, p=action_values)
            return action, action_values
        else:
            # take the action with the highest probability
            action = np.argmax(action_values)
            one_hot_action_probabilities = np.zeros(len(self.action_space.actions))
            one_hot_action_probabilities[action] = 1

            return action, one_hot_action_probabilities

    def get_control_param(self):
        return 0
