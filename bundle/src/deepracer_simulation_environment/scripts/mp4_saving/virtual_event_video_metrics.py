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

import numpy as np
import rospy

from collections import OrderedDict
from threading import RLock
from mp4_saving.constants import PROGRESS_INTERVAL
from markov.virtual_event.constants import DEFAULT_RACE_DURATION


class VirtualEventVideoMetrics():
    """
    VirtualEventVideoMetrics class
    """
    def __init__(self):
        """
        VirtualEventVideoMetrics constructor
        """
        self._lock = RLock()
        self.reset()

    @property
    def sim_time(self):
        """
        sim time getter

        Returns:
            float: sim time in ms
        """
        with self._lock:
            return self._sim_time

    @property
    def time_to_leader(self):
        """
        time to leader dict getter

        Returns:
            dict[str, float]: time to leader dict with key as racer name and value as gap time
        """
        with self._lock:
            return self._time_to_leader

    @property
    def lap(self):
        """
        lap getter

        Returns:
            int: lap count
        """
        with self._lock:
            return self._lap

    def update(self, racer_metrics):
        """
        update and calculate param

        Args:
            racer_metrics(dict[str, Metric]): racer_metrics with key as name and value as Metric
        """
        with self._lock:
            self._racer_metrics = racer_metrics
            if self._racer_metrics:
                self._racer_progresses = OrderedDict()
                self._time_to_leader = OrderedDict()
                self._calculate_sim_time()
                self._calculate_racer_progresses()
                self._calculate_lap()
                self._calculate_time_to_leader()

    def reset(self):
        """
        Reset video metric param
        """
        with self._lock:
            self._target_progress = PROGRESS_INTERVAL
            self._race_leader = None
            self._racer_metrics = OrderedDict()
            self._race_leader_time = list()
            self._race_leader_progress = list()
            self._time_to_leader = OrderedDict()
            self._racer_progresses = OrderedDict()
            self._lap = 0
            self._sim_time = 0.0
            self._race_duration = int(rospy.get_param("RACE_DURATION", DEFAULT_RACE_DURATION)) * 1000

    def _calculate_sim_time(self):
        """
        Calculate sim time
        """
        if self._race_leader:
            sim_time = self._racer_metrics[self._race_leader].total_evaluation_time
            self._sim_time = min(max(0.0, sim_time), self._race_duration)

    def _calculate_racer_progresses(self):
        """
        Calculate racer progresses
        """
        for agent_name, agent_metric in self._racer_metrics.items():
            progress = agent_metric.completion_percentage
            if progress > 0:
                progress %= 100.0
            self._racer_progresses.update(
                {agent_name: progress + 100.0 * agent_metric.lap_counter})

    def _calculate_lap(self):
        """
        Calculate lap count
        """
        racers_lap = list()
        for _, agent_metric in self._racer_metrics.items():
            racers_lap.append(agent_metric.lap_counter)
        self._lap = int(max(racers_lap)) if racers_lap else 0

    def _calculate_time_to_leader(self):
        """
        Calculate time to leader
        """
        self._race_leader = max(self._racer_progresses, key=self._racer_progresses.get)
        if self._racer_progresses[self._race_leader] > self._target_progress:
            self._target_progress += PROGRESS_INTERVAL
            self._race_leader_progress.append(self._racer_progresses[self._race_leader])
            self._race_leader_time.append(self._racer_metrics[self._race_leader].total_evaluation_time)
        for agent_name, agent_progress in self._racer_progresses.items():
            gap_to_leader = 0.0
            if self._race_leader_progress and \
                    self._race_leader_time and \
                    agent_name != self._race_leader:
                gap_to_leader = self._racer_metrics[self._race_leader].total_evaluation_time - np.interp(
                    agent_progress,
                    self._race_leader_progress,
                    self._race_leader_time)
            self._time_to_leader[agent_name] = gap_to_leader
