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

from markov.log_handler.deepracer_exceptions import GenericRolloutException
import threading
from markov.gazebo_tracker.abs_tracker import AbstractTracker


class EffectManager(AbstractTracker):
    """
    Effect Manager class that manages multiple effects
    """
    _instance_ = None

    @staticmethod
    def get_instance():
        """Method for getting a reference to the effect manager object"""
        if EffectManager._instance_ is None:
            EffectManager()
        return EffectManager._instance_

    def __init__(self):
        if EffectManager._instance_ is not None:
            raise GenericRolloutException("Attempting to construct multiple Effect Manager")
        self.effects = set()
        self.lock = threading.RLock()

        # there should be only one randomizer manager instance
        EffectManager._instance_ = self
        super(EffectManager, self).__init__()

    def update_tracker(self, delta_time, sim_time):
        """
        Callback when sim time is updated

        Args:
            delta_time (float): time diff from last call
            sim_time (Clock): simulation time
        """
        with self.lock:
            copy_effect_set = self.effects.copy()
            for effect in copy_effect_set:
                effect.update(delta_time)

    def add(self, effect):
        """
        Add effect to manager

        Args:
            effect (obj): the effect to add
        """
        with self.lock:
            self.effects.add(effect)
            effect.on_attach()

    def remove(self, effect):
        """
        Remove effect from manager

        Args:
            effect (obj): the effect to remove
        """
        with self.lock:
            self.effects.discard(effect)
            effect.on_detach()
