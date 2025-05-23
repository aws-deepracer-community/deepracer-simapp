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

'''This module implement sqs client'''

import logging
import botocore
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SQS_DELETE_MESSAGE_EXCEPTION,
                                          SIMAPP_SQS_RECEIVE_MESSAGE_EXCEPTION)
from markov.log_handler.logger import Logger
from markov.boto.constants import BotoClientNames
from markov.boto.deepracer_boto_client import DeepRacerBotoClient

LOG = Logger(__name__, logging.INFO).get_logger()


class SQSClient(DeepRacerBotoClient):
    """
    Connects to a FIFO SQS, retrieves messages in a batch and then deletes them
    """
    name = BotoClientNames.SQS.value

    def __init__(self, queue_url, region_name="us-east-1",
                 max_num_of_msg=1, wait_time_sec=5,
                 max_retry_attempts=3, backoff_time_sec=1.0,
                 session=None):
        """Initialize a sqs client with default exponital backoff retries.

        Args:
            queue_url (str): the queue url to receive message from.
            region_name (str, optional): the region name we want to create the client in.
                                         Defaults to "us-east-1".
            max_num_of_msg (int, optional): the max number of message we want to receive from the sqs queue.
                                            Defaults to 1.
            wait_time_sec (int, optional): The wait time in seconds we want to poll from sqs.
                                           Defaults to 5.
            max_retry_attempts (int, optional): The maxiumum retry attemps if something failed.
                                                Defaults to 5.
            backoff_time_sec (float, optional): The exponitial backoff time in seconds.
                                                Defaults to 1.0.
            session (boto3.Session): An alternative session to use.
                                     Defaults to None.

        """
        super(SQSClient, self).__init__(region_name=region_name,
                                        max_retry_attempts=max_retry_attempts,
                                        backoff_time_sec=backoff_time_sec,
                                        boto_client_name=self.name,
                                        session=session)
        self._queue_url = queue_url
        self._max_num_of_msg = max_num_of_msg
        self._wait_time_sec = wait_time_sec

    def get_messages(self):
        """Fetches the SQS messages.

        Returns:
            list(str): Strips out the Body section of each message and returns all of them in a list.
        """
        try:
            messages = self.exp_backoff(
                action_method=self.get_client().receive_message,
                QueueUrl=self._queue_url,
                AttributeNames=['SentTimestamp'],
                MessageAttributeNames=['All'],
                MaxNumberOfMessages=self._max_num_of_msg,
                WaitTimeSeconds=self._wait_time_sec)
            payload = []
            if messages.get('Messages'):
                entries = []
                for message in messages.get('Messages'):
                    payload.append(message['Body'])
                    entries.append({
                        'Id': message['MessageId'],
                        'ReceiptHandle': message['ReceiptHandle']
                    })
                self.delete_messages(entries)
                LOG.info("[sqs] Received payload %s", payload)
            return payload
        except botocore.exceptions.ClientError as ex:
            log_and_exit("[sqs] ClientError: Unable to receive message "
                         "from sqs queue {}: {}.".format(self._queue_url, ex),
                         SIMAPP_SQS_RECEIVE_MESSAGE_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)
        except Exception as ex:
            log_and_exit("[sqs] SystemError: Unable to receive message "
                         "from sqs queue {}: {}.".format(self._queue_url, ex),
                         SIMAPP_SQS_RECEIVE_MESSAGE_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)

    def _delete_messages(self, entries):
        """
        private delete message that deletes a group of messages from the SQS

        Args:
            entries ([dict]): A list of the messages dict to be added.
                              Each entry defines the message to be deleted by defining
                              the Id and the Receipt Handler from the original message.

        """
        try:
            resp = self.get_client().delete_message_batch(
                QueueUrl=self._queue_url,
                Entries=entries)
        except botocore.exceptions.ClientError as ex:
            raise Exception("Exceptions in deleting message"
                            "from sqs queue: {}, {}".format(self._queue_url,
                                                            ex)) from ex
        except Exception as ex:
            raise Exception("Exceptions in deleting message "
                            "from sqs queue: {}, {}".format(self._queue_url,
                                                            ex)) from ex
        if 'Successful' not in resp or \
                len(resp['Successful']) != len(entries):
            message = resp['Failed'] if "Failed" in resp["Failed"] and resp["Failed"] else \
                "delete message failed without error message"
            raise Exception("Exceptions in deleting message "
                            "from sqs queue {}, {}".format(self._queue_url,
                                                           message))

    def delete_messages(self, entries):
        """
        Public delete message that deletes a group of messages from the SQS with retry

        Args:
            entries ([dict]): A list of the messages dict to be added.
                              Each entry defines the message to be deleted by defining
                              the Id and the Receipt Handler from the original message.

        """
        self.exp_backoff(
            action_method=self._delete_messages,
            entries=entries)
