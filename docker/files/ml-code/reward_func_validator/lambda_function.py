# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

import json
import logging

logger = logging.getLogger()
logger.setLevel(logging.INFO)


def lambda_handler(event, _context):
    from validator import get_validation_response

    logger.info("Event: " + json.dumps(event))
    response = {
        "statusCode": 200,
        "body": json.dumps(
            get_validation_response(event["reward_function"], event["track_name"])
        ),
    }
    return response
