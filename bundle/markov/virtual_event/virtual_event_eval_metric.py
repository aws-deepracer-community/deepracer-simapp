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

from markov.metrics.constants import MetricsS3Keys
from markov.metrics.s3_metrics import EvalMetrics
from markov.virtual_event.constants import PAUSE_TIME_BEFORE_START


class VirtualEventEvalMetric():
    """
    VirtualEventEvalMEtrics class
    """
    def __init__(self,
                 agent_name,
                 race_data):
        """
        VirtualEventEvalMetric constructor

        Args:
            agent_name (str): list of agent names
            race_data (VirtualEventRaceData): VirtualEventRaceData class instance
        """
        self._agent_name = agent_name
        self._race_data = race_data
        dummy_metrics_s3_config = {MetricsS3Keys.METRICS_BUCKET.value: "dummy-bucket",
                                   MetricsS3Keys.METRICS_KEY.value: "dummy-key",
                                   MetricsS3Keys.REGION.value: self._race_data.region}
        self._eval_metric = \
            EvalMetrics(agent_name=agent_name,
                        s3_dict_metrics=dummy_metrics_s3_config,
                        is_continuous=self._race_data.is_continuous,
                        pause_time_before_start=PAUSE_TIME_BEFORE_START)

    @property
    def eval_metric(self):
        """
        Returns:
            EvalMetrics: evaluation metrics
        """
        return self._eval_metric

    def reset(self, profile, is_saving_simtrace):
        """
        Reset metric

        Args:
            profile (object): racer profile object
            is_saving_simtrace (bool): True if saving simtrace, False otherwise
        """
        metrics_s3_config = {MetricsS3Keys.METRICS_BUCKET.value: profile.outputMetrics.s3BucketName,
                             MetricsS3Keys.METRICS_KEY.value: profile.outputMetrics.s3KeyPrefix,
                             MetricsS3Keys.REGION.value: self._race_data.region}
        self._eval_metric.reset_metrics(
            s3_dict_metrics=metrics_s3_config,
            is_save_simtrace_enabled=is_saving_simtrace)
