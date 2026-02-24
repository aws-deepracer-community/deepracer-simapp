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

'''Concrete agent implementation for the DeepRacer Gazebo environment.

An ``Agent`` bundles a composite sensor and an agent controller.  It acts
as the internal glue between the sensor data pipeline and the Gazebo/ROS
action publishers, hiding those details from ``DeepRacerEnv``.
'''
from markov.sensors.constants import Input


class Agent(object):
    '''Bundles a composite sensor and a car-control object.

    The gymnasium ``DeepRacerEnv`` calls the methods on this class
    rather than talking to sensors and controllers directly.
    '''

    def __init__(self, sensor, ctrl):
        '''Construct an agent.

        Args:
            sensor: A :class:`~markov.sensors.composite_sensor.CompositeSensor`
                    (or any ``SensorInterface`` implementation).
            ctrl: An ``AgentCtrlInterface`` implementation (typically
                  :class:`~markov.agent_ctrl.rollout_agent_ctrl.RolloutCtrl`).
        '''
        self._sensor_ = sensor
        self._ctrl_   = ctrl

    # ------------------------------------------------------------------
    # Properties forwarded from the controller
    # ------------------------------------------------------------------

    @property
    def ctrl(self):
        return self._ctrl_

    # ------------------------------------------------------------------
    # Gymnasium-facing helpers
    # ------------------------------------------------------------------

    def get_observation_space(self):
        '''Return a ``gymnasium.spaces.Dict`` covering all active sensors.

        Delegates to the sensor\'s :meth:`get_observation_space`.
        '''
        if self._sensor_ is not None:
            return self._sensor_.get_observation_space()
        return None

    def get_action_space(self):
        '''Return the gymnasium action space exposed by the controller.'''
        return self._ctrl_.action_space

    # ------------------------------------------------------------------
    # Episode control
    # ------------------------------------------------------------------

    def reset_agent(self):
        '''Reset sensor buffers and move the car to the start position.

        Returns:
            dict | None: initial sensor observation, or None when no sensor
            is configured.
        '''
        sensor_state = None
        if self._sensor_ is not None:
            self._sensor_.reset()
            sensor_state = self._sensor_.get_state()
        self._ctrl_.reset_agent()
        return sensor_state

    def finish_episode(self):
        '''Run end-of-episode housekeeping in the controller.'''
        self._ctrl_.finish_episode()

    # ------------------------------------------------------------------
    # Step-level control
    # ------------------------------------------------------------------

    def send_action(self, action):
        '''Publish *action* to Gazebo via ROS.

        Args:
            action (np.ndarray): ``[steering_deg, speed_m_s]`` for the
                continuous action space, or an integer index for discrete.
        '''
        self._ctrl_.send_action(action)

    def update_agent(self, action):
        '''Update the agent\'s internal state after the action has been sent.

        Args:
            action: Same type as passed to :meth:`send_action`.

        Returns:
            dict: ``{agent_name: agent_info}`` mapping.
        '''
        return self._ctrl_.update_agent(action)

    def judge_action(self, action, agents_info_map):
        '''Compute the next observation, reward, and done flag.

        Args:
            action: Last action taken.
            agents_info_map (dict): All agents\' current info (needed by
                multi-agent reset rules).

        Returns:
            tuple(dict, float, bool): ``(next_observation, reward, done)``.
        '''
        next_state = self._sensor_.get_state() if self._sensor_ is not None else None
        reward, done, _ = self._ctrl_.judge_action(agents_info_map)

        # Optional visualisation overlay (no-op when the controller does not
        # expose a reward_data_pub).
        if hasattr(self._ctrl_, 'reward_data_pub') and self._ctrl_.reward_data_pub is not None:
            raw_state = self._sensor_.get_raw_state()
            for key in (Input.CAMERA.value, Input.OBSERVATION.value):
                if raw_state.get(key) is not None:
                    self._ctrl_.reward_data_pub.publish_frame(raw_state[key], action, reward)
                    break

        return next_state, reward, done
