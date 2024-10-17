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
import os
import rospy
from markov import utils
from markov.gazebo_utils.model_updater import ModelUpdater
from markov.virtual_event.virtual_event import VirtualEvent
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_WORKER_EXCEPTION)


LOG = Logger(__name__, logging.INFO).get_logger()


def main():
    """
    Main function for virutal event
    """
    ModelUpdater.get_instance().pause_physics()
    virtual_event = VirtualEvent()
    LOG.info("[VirtualEventWorker] virtual event start.")

    while True:
        if not virtual_event.is_event_end:
            virtual_event.poll()
        if virtual_event.is_event_end:
            break
        if virtual_event.setup():
            virtual_event.start()
            virtual_event.finish()

    LOG.info("[VirtualEventWorker] virtual event end.")
    utils.stop_ros_node_monitor()


if __name__ == '__main__':
    try:
        rospy.init_node('virtual_event_manager', anonymous=True)
        main()
    except Exception as ex:
        log_and_exit("Virtual event worker error: {}".format(ex),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)
