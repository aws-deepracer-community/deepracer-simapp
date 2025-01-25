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

from typing import Type

from rl_coach.architectures.head_parameters import HeadParameters


class SACQHeadParameters(HeadParameters):
    def __init__(self, activation_function: str = 'relu', name: str = 'sac_q_head_params', dense_layer=None,
                 layers_sizes: tuple = (256, 256), output_bias_initializer=None):
        super().__init__(parameterized_class_name='SACQHead', activation_function=activation_function, name=name,
                         dense_layer=dense_layer)
        self.network_layers_sizes = layers_sizes
        self.output_bias_initializer = output_bias_initializer
        self.P_action = None

    @property
    def path(self):
        return 'markov.multi_agent_coach.architectures.tensorflow_components.heads:' + self.parameterized_class_name


class SACPolicyHeadParameters(HeadParameters):
    def __init__(self, activation_function: str = 'relu', name: str = 'sac_policy_head_params',
                 dense_layer=None, sac_alpha=0.2, rescale_action_values=False, log_std_bounds=[-20, 2]):
        super().__init__(parameterized_class_name='SACPolicyHead', activation_function=activation_function, name=name,
                         dense_layer=dense_layer)
        self.sac_alpha = sac_alpha
        self.rescale_action_values = rescale_action_values
        self.log_std_bounds = log_std_bounds

    @property
    def path(self):
        return 'markov.multi_agent_coach.architectures.tensorflow_components.heads:' + self.parameterized_class_name
