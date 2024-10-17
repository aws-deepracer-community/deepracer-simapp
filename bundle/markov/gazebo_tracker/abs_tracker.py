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

import abc
from markov.gazebo_tracker.tracker_manager import TrackerManager
import markov.gazebo_tracker.constants as consts

# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})


class AbstractTracker(ABC):
    def __init__(self, priority=consts.TrackerPriority.NORMAL):
        self._priority = priority
        TrackerManager.get_instance().add(self, priority)

    @abc.abstractmethod
    def update_tracker(self, delta_time, sim_time):
        """
        Update Tracker

        Args:
            delta_time (float): delta time
            sim_time (Clock): simulation time
        """
        raise NotImplementedError('Tracker must be able to update')
