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

import pytest
from markov.tests import test_constant

@pytest.fixture
def aws_region():
    return test_constant.AWS_REGION

@pytest.fixture
def model_metadata_s3_key():
    return test_constant.MODEL_METADATA_S3_KEY

@pytest.fixture
def reward_function_s3_source():
    return test_constant.REWARD_FUNCTION_S3_SOURCE

@pytest.fixture
def s3_bucket():
    return test_constant.S3_BUCKET

@pytest.fixture
def s3_prefix():
    return test_constant.S3_PREFIX
