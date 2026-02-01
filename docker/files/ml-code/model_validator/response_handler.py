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

import logging
import json

logger = logging.getLogger(__name__)


UNKNOWN_ERROR = "Transient error, Please try again."
UNSUPPORTED_TYPE_ERROR_FORMAT = 'This only supports application/json data. Unsupported data type: {}'
MISSING_REQUIRED_ARGUMENTS_ERROR = "Missing required arguments: {}"

class ResponseHandler(object):
    @staticmethod
    def valid():
        return {
            "statusCode": 200,
            "body": json.dumps("valid"),
            "headers": {
                "Content-Type": "application/json"
            }
        }

    @staticmethod
    def unsupported_datatype(e=None, session_info=None):
        # 4xx errors
        logger.warning(
            "The content type, {}, is not supported. [session_info={}]"
            .format(e, session_info))
        message = UNSUPPORTED_TYPE_ERROR_FORMAT.format(e)
        return {
            "statusCode": 415,
            "body": json.dumps(message),
            "headers": {
                "Content-Type": "application/json"
            }
        }

    @staticmethod
    def invalid_model(e=None, session_info=None):
        logger.warning("Invalid model : {} [session_info={}]".format(e, session_info))
        return {
            "statusCode": 400,
            "body": json.dumps(str(e)),
            "headers": {
                "Content-Type": "application/json"
            }
        }

    @staticmethod
    def simapp_error(e=None, session_info=None):
        logger.error("An error occured during SimApp validation: {} [session_info={}]".format(e, session_info))
        return {
            "statusCode": 500,
            "body": json.dumps(str(e)),
            "headers": {
                "Content-Type": "application/json"
            }
        }

    @staticmethod
    def argument_error(e=None, session_info=None):
        logger.warning("Missing argument: {} [session_info={}]".format(e, session_info))
        return {
            "statusCode": 500,
            "body": json.dumps(MISSING_REQUIRED_ARGUMENTS_ERROR.format(e)),
            "headers": {
                "Content-Type": "application/json"
            }
        }

    @staticmethod
    def server_error(e=None, session_info=None):
        # 5XX errors
        logger.error("An error occurred during invoking Lambda: {} [session_info={}]".format(e, session_info))
        return {
            "statusCode": 500,
            "body": json.dumps(str(e)),
            "headers": {
                "Content-Type": "application/json"
            }
        }