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

# Logger class definition
import logging
import sys


class Logger(object):
    """
    Logger class for all DeepRacer Simulation Application logging
    """
    def __init__(self, logger_name, log_level=logging.INFO):
        self.logger = logging.getLogger(logger_name)
        self.logger.setLevel(log_level)

        handler = logging.StreamHandler()
        handler.setLevel(log_level)
        self.logger.addHandler(handler)

    def get_logger(self):
        """
        Returns the logger object with all the required log settings.
        """
        return self.logger
