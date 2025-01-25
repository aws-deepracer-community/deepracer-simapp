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

'''This module defines the concrete classes for training'''
from markov.agent_ctrl.utils import load_action_space
from markov.agent_ctrl.agent_ctrl_interface import AgentCtrlInterface

class TrainingCtrl(AgentCtrlInterface):
    '''Concrete class for an agent that drives forward'''
    def __init__(self, agent_name, model_metadata):
        """constructor for the training agent ctrl

        Args:
            agent_name (str): name of the agent
            model_metadata (ModelMetadata): object containing the details in the model metadata json file
        """
        # Store the name of the agent used to set agents position on the track
        self._agent_name_ = agent_name
        #Create default reward parameters
        self._action_space_ = load_action_space(model_metadata)
        self._model_metadata_ = model_metadata

    @property
    def action_space(self):
        return self._action_space_

    @property
    def model_metadata(self):
        return self._model_metadata_

    def reset_agent(self):
        pass

    def send_action(self, action):
        pass

    def update_agent(self, action):
        return {}

    def judge_action(self, agents_info_map):
        return None, None, None

    def finish_episode(self):
        pass
