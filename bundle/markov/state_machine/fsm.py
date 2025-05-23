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

"""this module implement fsm"""

from markov.state_machine.abs_fsm_state import AbsFSMState


class FSM(object):
    """
    Finite State machine base class
    """

    def __init__(self, initial_state):
        """ Initialize the components. """

        # Start state machine with an initial state
        self._current_state = initial_state

    @property
    def current_state(self):
        """return current state
        """
        return self._current_state

    @current_state.setter
    def current_state(self, val):
        """current state setter
        """
        if val is not None and not isinstance(val, AbsFSMState):
            raise ValueError("Invalid current state: {}".format(val))
        self._current_state = val

    @property
    def is_in_terminal(self):
        """return boolean on whether state machine is in termination state
        """
        return self._current_state is None

    def execute(self, input_val):
        """this is for finate state machin execute call

        Args:
            input_val (object): input values based on specific state machine
            for execution

        Returns:
            object: object return from state machine
        """

        # The next state will be the result of the on_event function.
        if self.is_in_terminal:
            return None
        self._current_state, ret_val = self._current_state.execute(input_val)
        if ret_val is None:
            return None
        # if ret_val is size 1, directly return the value out
        return ret_val[0] if len(ret_val) == 1 else ret_val
