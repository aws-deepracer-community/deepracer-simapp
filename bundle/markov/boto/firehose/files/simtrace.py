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

'''This module implements firehose client for simtrace'''

import logging
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                         SIMAPP_EVENT_ERROR_CODE_500)
from markov.boto.firehose.firehose_client import FirehoseClient
from markov.boto.s3.s3_client import S3Client

LOG = Logger(__name__, logging.INFO).get_logger()

# Firehose supports a maximum of 500 records per batch
MAX_RECORDS_PER_BATCH = 500

class FirehoseSimtrace:
    '''Firehose sim trace upload'''

    def __init__(self, delivery_stream_name, s3_bucket, s3_prefix, 
                 region_name='us-east-1', max_retry_attempts=5, 
                 backoff_time_sec=1.0):
        '''Firehose sim trace upload

        Args:
            delivery_stream_name (str): Firehose delivery stream name
            region_name (str): AWS region name
            max_retry_attempts (int): maximum retry attempts
            backoff_time_sec (float): retry backoff time in seconds
        '''
        if not delivery_stream_name:
            log_and_exit("Firehose simtrace delivery stream name not available.",
                         SIMAPP_SIMULATION_WORKER_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)
        self._delivery_stream_name = delivery_stream_name
        self._firehose_client = FirehoseClient(region_name=region_name,
                                               max_retry_attempts=max_retry_attempts,
                                               backoff_time_sec=backoff_time_sec)
        self._bucket = s3_bucket
        self._s3_prefix = s3_prefix
        self._s3_client = S3Client(region_name, 
                                   max_retry_attempts, 
                                   backoff_time_sec)
        self._buffer = []

    def add_record(self, record):
        '''Add a single sim trace record to the buffer

        Args:
            record (json): JSON text body in string format
        '''
        self._buffer.append({'Data': record + '\n'})
        if len(self._buffer) >= MAX_RECORDS_PER_BATCH: 
            self.persist_buffered_records()

    def persist_single_record(self, body):
        '''Upload a single sim trace record into Firehose delivery stream

        Args:
            body (json): text body in json format
        '''
        # if retry failed, firehose_client put_record will log and exit 500
        record = {'Data': body + '\n'}
        self._firehose_client.put_record(delivery_stream_name=self._delivery_stream_name,
                                         record=record)
        LOG.info("[firehose] Successfully uploaded simtrace to delivery stream {}."
                 .format(self._delivery_stream_name))

    def persist_buffered_records(self): 
        '''Upload buffered records to Firehose delivery stream'''
        # if retry failed, firehose_client put_record_batch will log and exit 500
        self._firehose_client.put_record_batch(delivery_stream_name=self._delivery_stream_name,
                                                        records=self._buffer)
        LOG.info("[firehose] Successfully uploaded simtrace to \
                firehose delivery stream {}.".format(self._delivery_stream_name))
        
        # Clear buffer after uploading to s3 bucket 
        self._buffer.clear()

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
                    LOG.info("[firehose] Successfully deleted metric object from \
                         s3 bucket {} with s3 key {}.".format(self._bucket,
                                                              obj['Key']))
