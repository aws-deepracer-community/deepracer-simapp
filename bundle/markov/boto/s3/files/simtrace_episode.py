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

'''This module implements s3 client for simtrace episode-wise upload'''

import os
import logging

from markov.log_handler.logger import Logger
from markov.boto.s3.s3_client import S3Client
from markov.boto.s3.constants import SIMTRACE_EPISODE_POSTFIX_DICT

LOG = Logger(__name__, logging.INFO).get_logger()

class SimtraceEpisodeUpload():
    '''This class is for all s3 simtrace episode-wise upload
    '''
    def __init__(self, upload_type, bucket, s3_prefix, 
                 region_name="us-east-1",
                 local_path="./custom_files/episode_data/\
                 agent/file",  
                 max_retry_attempts=5, backoff_time_sec=1.0):
        '''This class is for all s3 simtrace episode-wise upload


        Args:
            upload_type (str): upload simtrace type
            bucket (str): S3 bucket string
            s3_prefix (str): S3 prefix string
            region_name (str): S3 region name
            local_path (str): file local path
            max_retry_attempts (int): maximum number of retry attempts for S3 download/upload
            backoff_time_sec (float): backoff second between each retry

        '''
        self._upload_type = upload_type
        self._bucket = bucket
        self._s3_key = os.path.normpath(os.path.join(
            s3_prefix, 
            SIMTRACE_EPISODE_POSTFIX_DICT[self._upload_type]))
        self._s3_prefix = s3_prefix
        self._local_path = local_path
        self._upload_num = 0
        self._s3_client = S3Client(region_name, 
                                   max_retry_attempts, 
                                   backoff_time_sec)

    def persist(self, s3_kms_extra_args):
        '''persist simtrace into s3 bucket

        Args:
            s3_kms_extra_args(dict): s3 key management service extra argument

        '''
        # upload sim trace
        # if retry failed, s3_client upload_file will log and exit 500
        self._s3_client.upload_file(bucket=self._bucket,
                                    s3_key=self._s3_key.format(self._upload_num),
                                    local_path=self._local_path,
                                    s3_kms_extra_args=s3_kms_extra_args)
        LOG.info("[s3_simtrace_episode] Successfully uploaded {} to \
             s3 bucket {} with s3 key {}.".format(self._upload_type,
                                                  self._bucket,
                                                  self._s3_key.format(self._upload_num)))
        self._upload_num += 1
        # remove local file after upload simtrace to s3 bucket
        os.remove(self._local_path)

    def delete(self):
        '''Delete simtrace objects from S3 bucket'''
        # delete sim trace
        # if retry failed, s3_client list_objects_v2 will log and exit 500
        paginator = self._s3_client.paginate(bucket=self._bucket, prefix=self._s3_prefix)
        for page in paginator:
            if 'Contents' in page:
                for obj in page['Contents']:
                    # if retry failed, s3_client delete_object will log and exit 500
                    self._s3_client.delete_object(bucket=self._bucket, s3_key=obj['Key'])
                    LOG.info("[s3_simtrace_episode] Successfully deleted {} from \
                         s3 bucket {} with s3 key {}.".format(self._upload_type,
                                                              self._bucket,
                                                              obj['Key']))
