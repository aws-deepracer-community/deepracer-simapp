# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

import os
import sys

sys.path.append(
    os.path.dirname(__file__)
)  # append current directory so relative imports can work

import datetime
import logging
import os

from constants import REWARD_FUNCTION_PATH, TRACK_NAME_PATH
from test_reward_function import run_suites

logger = logging.getLogger()
logger.setLevel(logging.INFO)


def get_validation_response(reward_function, track_name):
    try:
        # set the Content-Type header so that the browser is aware that the response
        # is formatted as JSON and our frontend JavaScript code is able to
        # appropriately parse the response.
        response = run_suites(reward_function, track_name)
        return response
    except Exception as e:
        return build_error_response(f"Exception occured during validation: {str(e)}")
    finally:
        # Making sure the temporary reward function and track name is deleted for next reqeust
        silentremove(REWARD_FUNCTION_PATH)
        silentremove(TRACK_NAME_PATH)


def build_error_response(msg):
    error = dict()
    error["date"] = str(datetime.datetime.now(datetime.timezone.utc))
    error["message"] = msg
    return [error]


def silentremove(filename):
    try:
        os.remove(filename)
    except Exception as e:
        logger.error(f"Exception occured during deletion: {str(e)}")
