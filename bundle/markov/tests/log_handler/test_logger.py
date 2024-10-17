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

""" The script takes care of testing the functionality of logger.py
"""
import pytest
import logging
from markov.log_handler.logger import Logger

@pytest.mark.robomaker
@pytest.mark.sagemaker
def test_get_logger():
    """The test function checks if the Logger is instantiated properly.
    """
    log = Logger(__name__, logging.INFO).get_logger()
    assert isinstance(log, logging.Logger)
