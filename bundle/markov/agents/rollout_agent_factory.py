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

'''This module is used to create agents for the rollout worker'''
from markov.agent_ctrl.bot_cars_agent_ctrl import  BotCarsCtrl
from markov.agent_ctrl.constants import ConfigParams
from markov.agent_ctrl.rollout_agent_ctrl import RolloutCtrl
from markov.agent_ctrl.obstacles_agent_ctrl import ObstaclesCtrl
from markov.agents.agent import Agent
from markov.agents.utils import construct_sensor, get_network_settings
from markov.sensors.sensors_rollout import SensorFactory
from markov.cameras.frustum_manager import FrustumManager
from markov.boto.s3.constants import ModelMetadataKeys
from markov.agents.llm_agent import LLMAgent
from markov.architecture.constants import NeuralNetwork


def create_rollout_agent(agent_config, metrics, run_phase_subject=None):
    """
    Factory method for creating agents based on the agent_config
    Args:
        agent_config (dict): agent configuration containing neural network type
        metrics: metrics object
        run_phase_subject: Subject that notifies observers when the run phase changes
    Returns:
        Agent object
    """
    # Get model metadata
    agent_name = agent_config['agent_name']  
    model_metadata = agent_config['model_metadata']
    model_metadata_info = model_metadata.get_model_metadata_info()
    observation_list = model_metadata_info[ModelMetadataKeys.SENSOR.value]
    network = model_metadata_info[ModelMetadataKeys.NEURAL_NETWORK.value]
    version = model_metadata_info[ModelMetadataKeys.VERSION.value]
    agent_name = agent_config[ConfigParams.CAR_CTRL_CONFIG.value][ConfigParams.AGENT_NAME.value]
    sensor = construct_sensor(agent_name, observation_list, SensorFactory, model_metadata_info)
    network_settings = get_network_settings(sensor, network)
    FrustumManager.get_instance().add(agent_name=agent_name,
                                        observation_list=observation_list,
                                        version=version)

    # Get neural network type from model metadata
    neural_network_type = model_metadata.get_model_metadata_info()[ModelMetadataKeys.NEURAL_NETWORK.value]
    
    # If LLM type, create LLM agent
    if neural_network_type == NeuralNetwork.LLM.value:
        sensor = construct_sensor(agent_name, model_metadata_info[ModelMetadataKeys.SENSOR.value],
                                SensorFactory, model_metadata_info)
        ctrl = RolloutCtrl(agent_config)
        return LLMAgent(sensor, ctrl, metrics, agent_name, run_phase_subject)
    else:
        # Existing code for regular neural network agents
        ctrl_config = agent_config[ConfigParams.CAR_CTRL_CONFIG.value]
        ctrl = RolloutCtrl(ctrl_config, run_phase_subject, metrics)

        return Agent(network_settings, sensor, ctrl)

def create_obstacles_agent():
    '''Returns an obstacle agent, such as a box. Will not be used for training'''
    ctrl = ObstaclesCtrl()
    return Agent(None, None, ctrl)

def create_bot_cars_agent(pause_time_before_start=0.0):
    '''Returns a bot car agent. Will not be used for training'''
    ctrl = BotCarsCtrl(pause_time_before_start=pause_time_before_start)
    return Agent(None, None, ctrl)
