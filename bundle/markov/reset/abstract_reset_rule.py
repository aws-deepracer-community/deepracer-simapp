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

'''This class defines an interface for how agents reset with the environment.
'''
import abc
from markov.log_handler.deepracer_exceptions import GenericRolloutException
class AbstractResetRule(object, metaclass=abc.ABCMeta):
    def __init__(self, name):
        if not name or not isinstance(name, str):
            raise GenericRolloutException("reset name cannot be None or empty string")
        self._name = name
        self._done = False

    def update(self, agent_status):
        '''update specific reset rule done flag

        Args:
            agent_status (dict): agent control status dictionary

        Returns:
            dict: dictionary contains single agent info after update

        '''
        ret_val = self._update(agent_status)
        return ret_val if ret_val else {}

    @abc.abstractmethod
    def _update(self, agent_status):
        '''update specific reset rule done flag

        Args:
            agent_status (dict): agent control status dictionary

        Raises:
            NotImplementedError: Reset class must be able to update rules
        '''
        raise NotImplementedError('Reset class must be able to update rules')

    @property
    def name(self):
        '''Get the reset rule name
        '''
        return self._name

    @property
    def done(self):
        '''Get the reset rule done flag
        '''
        return self._done

    def reset(self):
        '''Set the reset rule done flag
        '''
        self._done = False
