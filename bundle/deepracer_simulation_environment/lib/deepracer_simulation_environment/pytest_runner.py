#!/usr/bin/env python
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
