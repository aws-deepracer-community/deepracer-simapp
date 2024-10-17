#!/usr/bin/env python3

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

from __future__ import print_function

import os
import sys
import rospy
import pytest


def get_output_file():
    for arg in sys.argv:
        if arg.startswith('--gtest_output'):
            return arg.split('=xml:')[1]

    raise RuntimeError('No output file has been passed')


if __name__ == '__main__':
    output_file = get_output_file()
    test_module = rospy.get_param('test_module')
    runner_path = os.path.dirname(os.path.realpath(__file__))
    module_path = os.path.join(runner_path, test_module)

    sys.exit(
        pytest.main([module_path, '--junitxml={}'.format(output_file),
                     '--cov={}/../scripts/'.format(runner_path)])
    )
