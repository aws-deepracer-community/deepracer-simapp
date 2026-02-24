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
from unittest.mock import MagicMock, patch

# rosnode is imported in NodeMonitor class, but this package is not a ROS environment.
# Since this is missing the rosnode, mocking the rosnode module.
# https://stackoverflow.com/questions/8658043/how-to-mock-an-import
import sys
sys.modules['rosnode'] = MagicMock()
sys.modules['botocore.config'] = MagicMock()
sys.modules['boto3'] = MagicMock()
sys.modules['markov.log_handler.constants'] = MagicMock()
sys.modules['markov.log_handler.exception_handler'] = MagicMock()

from deepracer_node_monitor.aws_utils.job_utils import JobUtils


@patch("deepracer_node_monitor.aws_utils.job_utils.os")
class JobUtilsTest(TestCase):
    def test_get_simulation_arn(self, os_mock):
        os_mock.environ.get.return_value = "ROBOMAKER_ARN"
        self.assertEqual("ROBOMAKER_ARN", JobUtils.get_simulation_arn())

    def test_get_simulation_arn_empty(self, os_mock):
        os_mock.environ.get.return_value = ''
        self.assertEqual("", JobUtils.get_simulation_arn())

    @patch("deepracer_node_monitor.aws_utils.job_utils.JobUtils.get_simulation_arn")
    def test_get_job_id(self, get_simulation_arn_mock, os_mock):
        get_simulation_arn_mock.return_value = "arn:aws:robomaker:us-east-1:687392285187:simulation-job/sim-n3vp4ydpwkx8"
        self.assertEqual("sim-n3vp4ydpwkx8", JobUtils.get_job_id())

    def test_get_aws_region(self, os_mock):
        os_mock.environ.get.return_value = "us-east-1"
        self.assertEqual("us-east-1", JobUtils.get_aws_region())
