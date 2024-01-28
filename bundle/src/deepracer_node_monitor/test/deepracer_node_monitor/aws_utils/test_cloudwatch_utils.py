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
from unittest import TestCase
from unittest.mock import MagicMock

# rosnode is imported in NodeMonitor class, but this package is not a ROS environment.
# Since this is missing the rosnode, mocking the rosnode module.
# https://stackoverflow.com/questions/8658043/how-to-mock-an-import
import sys
sys.modules['rosnode'] = MagicMock()
sys.modules['botocore'] = MagicMock()
sys.modules['boto3'] = MagicMock()
sys.modules['markov.log_handler.constants'] = MagicMock()
sys.modules['markov.log_handler.exception_handler'] = MagicMock()

from deepracer_node_monitor.aws_utils.cloudwatch_utils import CloudWatchJobStatusMetricDimensionData


class CloudWatchJobStatusMetricDimensionDataTest(TestCase):
    def setUp(self) -> None:
        self.job_status = "SUCCESS"
        self.simapp_id = "SIMAPP_ID"

    def test_initialize(self):
        dimenstion_data = CloudWatchJobStatusMetricDimensionData(job_status=self.job_status)
        self.assertEqual(dimenstion_data._job_status, self.job_status)
        self.assertEqual(dimenstion_data._simapp_id, None)
        dimenstion_data = CloudWatchJobStatusMetricDimensionData(job_status=self.job_status)
        self.assertEqual(dimenstion_data._job_status, self.job_status)
        self.assertEqual(dimenstion_data._simapp_id, None)
        dimenstion_data = CloudWatchJobStatusMetricDimensionData(job_status=self.job_status, simapp_id=self.simapp_id)
        self.assertEqual(dimenstion_data._job_status, self.job_status)
        self.assertEqual(dimenstion_data._simapp_id, self.simapp_id)

    def test_job_status_Property(self):
        dimenstion_data = CloudWatchJobStatusMetricDimensionData(job_status=self.job_status)
        self.assertEqual(dimenstion_data.job_status, self.job_status)

    def test_simapp_id_Property(self):
        dimenstion_data = CloudWatchJobStatusMetricDimensionData(job_status=self.job_status, simapp_id=self.simapp_id)
        self.assertEqual(dimenstion_data.simapp_id, self.simapp_id)

    def test_to_cloudwatch_dict_with_only_jobstatus(self):
        return_value = {
            "MetricName": "SUCCESS",
            "Dimensions": [{'Name': 'Workflow', 'Value': 'EVALUATION_START'}],
            "Value": 1,
            "Unit": "Count"
        }
        dimenstion_data = CloudWatchJobStatusMetricDimensionData(job_status=self.job_status)
        self.assertEqual(return_value, dimenstion_data.to_cloudwatch_dict())

    def test_to_cloudwatch_dict_with_jobstatus_simappid(self):
        return_value = {
            "MetricName": "SUCCESS",
            "Dimensions": [{'Name': 'Workflow', 'Value': 'EVALUATION_START'},
                           {'Name': 'SimAppId', 'Value': 'SIMAPP_ID'}],
            "Value": 1,
            "Unit": "Count"
        }
        dimenstion_data = CloudWatchJobStatusMetricDimensionData(job_status=self.job_status, simapp_id=self.simapp_id)
        self.assertEqual(return_value, dimenstion_data.to_cloudwatch_dict())
