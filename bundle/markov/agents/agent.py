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

'''This module contains the concrete implementations of the agent interface'''
from markov.architecture.constants import Input

class Agent(object):
    '''Concrete class for agents running in the rollout worker'''
    def __init__(self, network_settings, sensor, ctrl):
        '''network_settings - Dictionary containing the desired
                              network configuration
           sensor - Reference to the composite sensor object
           ctrl - Reference to the car control object
           metrics - Reference to the metrics object
        '''
        self._network_settings_ = network_settings
        self._sensor_ = sensor
        self._ctrl_ = ctrl

    @property
    def network_settings(self):
        return self._network_settings_

    @property
    def ctrl(self):
        return self._ctrl_

    def get_observation_space(self):
        '''Get the sensor obervation space
        '''
        if self._sensor_ is not None:
            return self._sensor_.get_observation_space()

    def get_action_space(self):
        '''Get the control action space
        '''
        return self._ctrl_.action_space

    def reset_agent(self):
        '''Reset agent control and metric instance

        The order is important for virtual event with
        dynamic spawning, so do not change order.

        During virtual event, agent might be deleted and respawn.
        It might take a while for sensor topic to be ready. Therefore,
        it is required to reset agent which reset all internal timer
        for virtual event after sensor reset which wait for sensor topic
        to be alive
        '''
        sensor_state = None
        if self._sensor_ is not None:
            self._sensor_.reset()
            sensor_state = self._sensor_.get_state()
        self._ctrl_.reset_agent()
        return sensor_state

    def finish_episode(self):
        ''' Finish episode and update its metrics into s3 bucket
        '''
        self._ctrl_.finish_episode()

    def send_action(self, action):
        '''Publish action index to gazebo

        Args:
            action (int or list): model metadata action_space index for discreet action spaces
                                  or [steering, speed] float values for continuous action spaces
        '''
        self._ctrl_.send_action(action)

    def update_agent(self, action):
        '''Update agent status based on taken action and env change

        Args:
            action: Interger with the desired action to take

        Returns:
            dict: dictionary contains single agent info map after desired action is taken
                  with key as each agent's name and value as each agent's info
        '''
        return self._ctrl_.update_agent(action)

    def judge_action(self, action, agents_info_map):
        '''Judge the taken action

        Args:
            action: Interger with the desired action to take
            agents_info_map: dictionary contains all agents info map with key as
                             each agent's name and value as each agent's info

        Returns:
            tuple: tuple contains next state sensor observation, reward value, and done flag
        '''
        if self._sensor_ is not None:
            next_state = self._sensor_.get_state()
        else:
            next_state = None
        reward, done, _ = self._ctrl_.judge_action(agents_info_map)
        if hasattr(self._ctrl_, 'reward_data_pub') and self._ctrl_.reward_data_pub is not None:
            raw_state = self._sensor_.get_raw_state()
            # More visualizations topics can be added here
            if Input.CAMERA.value in raw_state and raw_state[Input.CAMERA.value] is not None:
                self._ctrl_.reward_data_pub.publish_frame(raw_state[Input.CAMERA.value], action, reward)
            elif Input.OBSERVATION.value in raw_state and raw_state[Input.OBSERVATION.value] is not None:
                self._ctrl_.reward_data_pub.publish_frame(raw_state[Input.OBSERVATION.value], action, reward)
        return next_state, reward, done
