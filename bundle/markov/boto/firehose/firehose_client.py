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

'''This module implements firehose client'''

import botocore
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.deepracer_exceptions import GenericNonFatalException
from markov.log_handler.constants import (SIMAPP_EVENT_SYSTEM_ERROR,
                                         SIMAPP_EVENT_USER_ERROR,
                                         SIMAPP_EVENT_ERROR_CODE_500,
                                         SIMAPP_FIREHOSE_DATA_STORE_EXCEPTION)
from markov.boto.constants import BotoClientNames
from markov.boto.deepracer_boto_client import DeepRacerBotoClient

class FirehoseClient(DeepRacerBotoClient):
    """Firehose Boto Client"""
    name = BotoClientNames.FIREHOSE.value

    def __init__(self, region_name="us-east-1", max_retry_attempts=5,
                 backoff_time_sec=1.0, session=None):
        """Firehose client

        Args:
            region_name (str): aws region name
            max_retry_attempts (int): maximum number of retry
            backoff_time_sec (float): backoff second between each retry
            session (boto3.Session): An alternative session to use.
                                     Defaults to None.
        """
        super(FirehoseClient, self).__init__(region_name=region_name,
                                             max_retry_attempts=max_retry_attempts,
                                             backoff_time_sec=backoff_time_sec,
                                             boto_client_name=self.name,
                                             session=session)

    def put_record(self, delivery_stream_name, record):
        """Put a single record into Firehose

        Args:
            delivery_stream_name (str): name of the Firehose delivery stream
            record (dict): record to be sent
        """
        try:
            self.exp_backoff(
                action_method=self.get_client().put_record,
                DeliveryStreamName=delivery_stream_name,
                Record=record)
        except botocore.exceptions.ClientError as err:
            log_and_exit("Unable to put record to Firehose: stream: {}, error: {}"
                         .format(delivery_stream_name, err.response['Error']['Code']),
                         SIMAPP_FIREHOSE_DATA_STORE_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)
        except Exception as ex:
            log_and_exit("Exception in putting record: {}".format(ex),
                         SIMAPP_FIREHOSE_DATA_STORE_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)

    def put_record_batch(self, delivery_stream_name, records):
        """Put multiple records into Firehose

        Args:
            delivery_stream_name (str): name of the Firehose delivery stream
            records (list): list of records to be sent
        """
        try:
            response = self.exp_backoff(
                action_method=self.get_client().put_record_batch,
                DeliveryStreamName=delivery_stream_name,
                Records=records
            )

            failed_count = response.get('FailedPutCount', 0)
            if failed_count > 0:
                error_msg = "{} records failed to process in stream {}.".format(failed_count, 
                                                                                delivery_stream_name)
                failed_records = [
                    "Record failed: {}, ErrorCode: {}, ErrorMessage: {}".format(records[i],
                                                                                res.get('ErrorCode'),
                                                                                res.get('ErrorMessage'))
                    for i, res in enumerate(response['RequestResponses']) if 'ErrorCode' in res
                ] 
                for msg in failed_records:
                    error_msg += "\n" + msg
                    raise GenericNonFatalException(error_msg=error_msg,
                                                   error_code=SIMAPP_FIREHOSE_DATA_STORE_EXCEPTION,
                                                   error_name=SIMAPP_EVENT_ERROR_CODE_500)
        except botocore.exceptions.ClientError as err:
            log_and_exit("Unable to put records to Firehose: stream: {}, error: {}".format(delivery_stream_name, 
                                                                                           err.response['Error']['Code']),
                         SIMAPP_FIREHOSE_DATA_STORE_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)
        except Exception as ex:
            log_and_exit("Exception in putting records: {}".format(ex),
                         SIMAPP_FIREHOSE_DATA_STORE_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)
