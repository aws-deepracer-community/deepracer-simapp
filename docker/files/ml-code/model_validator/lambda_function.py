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

import os
import shutil
import json
import subprocess
import uuid
import logging

from utils import build_validation_worker_cmd, run_cmd
from response_handler import ResponseHandler
response_handler = ResponseHandler

# The logger object
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s.%(msecs)03d %(levelname)s %(module)s - %(funcName)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

MODEL_DATA_PATH = '/tmp/model_data'
S3_KMS_CMK_ARN_ENV = 'S3_KMS_CMK_ARN_ENV'
IS_MODEL_VALIDATION = 'IS_MODEL_VALIDATION'


def lambda_handler(event, context):
    """Validate user trained model located at given S3 location.
    """
    try:
        # Fetch the session data
        session_info = dict()
        session_info['requestId'] = context.aws_request_id
        session_info['processId'] = os.getpid()
        s3_bucket = event.get('s3_bucket')
        s3_prefix = event.get('s3_prefix')
        aws_region = event.get('aws_region')
        if not s3_bucket or not s3_prefix or not aws_region:
            raise Exception("s3_bucket, s3_prefix or aws_region not passed in.")
        subprocess_env = os.environ.copy()
        if 'sse_key_id' in event and event.get('sse_key_id'):
            subprocess_env[S3_KMS_CMK_ARN_ENV] = event.get('sse_key_id')
            logger.info("S3_KMS_CMK_ARN_ENV set as {}".format(event.get('sse_key_id')))
        # This is required to have a separate flow during log_and_exit for model validation subprocess
        # such that we do not write any physical sync files on disk and return the subprocess execution
        # with an appropriate return code 0/1 based on success / failure.
        subprocess_env[IS_MODEL_VALIDATION] = "True"
    except Exception as ex:
        return response_handler.argument_error(ex, session_info)

    try:
        # Create a separate directory for every model validation call
        custom_files_path = os.path.join(MODEL_DATA_PATH, session_info['requestId'])
        if not os.path.exists(custom_files_path):
            logger.info("Creating custom file path {} for the model validation request {}".format(custom_files_path, session_info))
            os.makedirs(custom_files_path)
        else:
            raise Exception("Custom Files Path already exists!: {}".format(custom_files_path))
        # Build the validation worker cmd
        validator_cmd = build_validation_worker_cmd(s3_bucket=s3_bucket, s3_prefix=s3_prefix, aws_region=aws_region)
        logger.info("Executing {} [session_info={}]".format(" ".join(map(str, validator_cmd)), session_info))
        return_code, _, stderr = run_cmd(cmd_args=validator_cmd,
                                         change_working_directory=custom_files_path,
                                         shell=False,
                                         stdout=None,
                                         env=subprocess_env)
        stderr = stderr.decode("utf-8")
        stderr_lines = stderr.splitlines()
        msg = "Validator exit with return code {} and stderr: {}".format(return_code, stderr)
        logger.info("{} [session_info={}] ".format(msg, session_info))
        # The return code is set by simapp to 1 in case there is any exception logged during the code execution.
        # and the process is exited manually during the log_and_exit flow.
        if return_code != 0:
            # Parse through the stderr to match with the correct error classification.
            for line in stderr_lines:
                if 'simapp_exception' in line:
                    err_json_string = line
                    err_json = json.loads(err_json_string)
                    if err_json['simapp_exception']['errorCode'].startswith('4'):
                        logger.info("Returning invalid response due to {} [session_info={}] ".format(line, session_info))
                        return response_handler.invalid_model(err_json, session_info)
                    else:
                        logger.info("Returning simapp error response due to {} [session_info={}] ".format(line, session_info))
                        return response_handler.simapp_error(err_json, session_info)
            logger.info("Raising process failure [session_info={}]".format(session_info))
            raise Exception("Unhandled exception due to process failure: {}".format(stderr))
        logger.info("Returning valid model response [session_info={}]".format(session_info))
        return response_handler.valid()
    except Exception as ex:
        logger.error("Unknown Server Exception raised: {} [session_info={}]".format(ex, session_info))
        return response_handler.server_error(ex, session_info)
    finally:
        # Make sure the temporary folder is deleted,
        # when validation_work preemptively exited with sys.exit
        # without deletion of temporary folder.
        logger.info("Cleanup [session_info={}]".format(session_info))
        shutil.rmtree(custom_files_path, ignore_errors=True)