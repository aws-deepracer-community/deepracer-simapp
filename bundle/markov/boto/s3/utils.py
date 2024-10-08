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

'''This module implement s3 utils'''

import os

def get_s3_key(s3_prefix, postfix):
    '''Parse hyperparameters S3 prefix and postfix into key

    Args:
        s3_prefix(str): s3 prefix
        postfix(str): postfix

    Returns:
        str: s3 key by joining prefix and postfix

    '''

    # parse S3 prefix and postfix into key
    s3_key = os.path.normpath(os.path.join(s3_prefix, postfix))
    return s3_key

def is_power_of_two(n):
    """Return True if n is a power of two."""
    if n <= 0:
        return False
    else:
        return n & (n - 1) == 0
