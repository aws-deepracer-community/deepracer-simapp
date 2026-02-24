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

'''This module houses the constants for the boto client'''
from enum import Enum

BOTO_ERROR_MSG_FORMAT = "{0} failed, retry after {1} seconds. Re-try count: {2}/{3}: {4}"


class BotoClientNames(Enum):
    '''Enum contains  all boto client for DeepRacer SimApp
    '''
    S3 = "s3"
    SQS = "sqs"
    CLOUD_WATCH_LOGS = "logs"
    FIREHOSE = "firehose"
