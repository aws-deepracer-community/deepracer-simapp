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
from unittest.mock import MagicMock

# rclpy is imported in NodeMonitor class, but this package is not a ROS environment.
# Since this is missing the rclpy, mocking the rclpy module.
# https://stackoverflow.com/questions/8658043/how-to-mock-an-import
import sys

sys.modules["rclpy"] = MagicMock()
sys.modules["botocore"] = MagicMock()
sys.modules["boto3"] = MagicMock()
sys.modules["markov.log_handler.constants"] = MagicMock()
sys.modules["markov.log_handler.exception_handler"] = MagicMock()
sys.modules["markov.utils"] = MagicMock()
sys.modules["markov.constants"] = MagicMock()


def test_deepracer_node_monitor_importable():
    from deepracer_node_monitor import DeepRacerNodeMonitor  # noqa: F401
