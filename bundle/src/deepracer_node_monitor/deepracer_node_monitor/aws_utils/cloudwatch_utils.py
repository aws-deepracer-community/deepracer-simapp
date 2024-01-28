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
""" Class for the Cloudwatch job status metrics & Dimension data"""
import os
from typing import Dict, Optional, Any

from deepracer_node_monitor.constants import JobStatusMetricDimension
from deepracer_node_monitor.aws_utils.constants import CW_EVALUATION_METRIC_NAME, CW_TRAINING_METRIC_NAME


class CloudWatchJobStatusMetricDimensionData(object):
    def __init__(self, job_status: str, simapp_id: Optional[str] = None) -> None:
        """
        Data class to hold the JobStatus metric dimensions

        Args:
            job_status (str): Status of simulator FAILED, INITIALIZING, RUNNING
            simapp_id (Optional[str]): RoboMaker simulation ID
        """
        self._job_status = job_status
        self._simapp_id = simapp_id

    @staticmethod
    def is_cloudwatch_heartbeat_enabled() -> bool:
        """
        Returns true if the cloudwatch heartbeat is value set is 1

        Returns
            bool: True if Environment variable set for HEARTBEAT_S3_LOCATION is 1
        """
        cw_heartbeat_env_val = os.environ.get('HEARTBEAT_CLOUDWATCH_METRICS', '')
        return cw_heartbeat_env_val == "1"

    @property
    def job_status(self) -> str:
        """
        Returns job status

        Returns:
            str: job status value
        """
        return self._job_status

    @property
    def simapp_id(self) -> str:
        """
        Returns simapp id

        Returns:
            str: simapp id value
        """
        return self._simapp_id

    def to_cloudwatch_dict(self) -> Dict[str, Any]:
        """
        Returns the cloud watch meteric data content for JobStatus metric.
        The return will contain dimensions if simapp_id are not none in dimenstion_data.

        Returns
            (Dict[str, Any]): Dict containing cloudwatch put_metric_data's metric information.
        """
        metric_name = CW_TRAINING_METRIC_NAME
        if(os.environ.get('SERVICE_TYPE', '') == 'SAGEMAKER' and os.environ.get('JOB_TYPE', '') == "training"):
            metric_name = CW_TRAINING_METRIC_NAME
        else:
            metric_name = CW_EVALUATION_METRIC_NAME

        dimensions = list()
        dimensions.append({"Name": JobStatusMetricDimension.WORKFLOW, "Value": metric_name})

        if self.simapp_id:
            dimensions.append({"Name": JobStatusMetricDimension.SIM_APP_ID, "Value": self.simapp_id})
        return {
            "MetricName": self.job_status,
            'Dimensions': dimensions,
            "Value": 1,
            "Unit": "Count"
        }
