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

'''This module should contain common utility methods for the rollout woeker that
   depend on ros, it should not be used in modules imported to the training worker
'''
import logging
import rclpy
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Empty
from markov.agents.utils import RunPhaseSubject
from markov.common import ObserverInterface
from markov.log_handler.logger import Logger
from markov.rclpy_wrappers import ServiceProxyWrapper
from markov.domain_randomizations.randomizer_manager import RandomizerManager
from markov.domain_randomizations.visual.light_randomizer import LightRandomizer
from markov.domain_randomizations.constants import GazeboServiceName
from markov.constants import (ROBOMAKER_IS_PROFILER_ON, ROBOMAKER_PROFILER_S3_BUCKET,
                              ROBOMAKER_PROFILER_S3_PREFIX)
from markov.utils import str2bool
from markov.world_config import WorldConfig
from deepracer_msgs.srv import GetLightNames

from rl_coach.core_types import RunPhase

LOG = Logger(__name__, logging.INFO).get_logger()
# Mapping of the phase to the text to display
PHASE_TO_MSG_MAP = {RunPhase.HEATUP : 0,
                    RunPhase.TRAIN : 1,
                    RunPhase.TEST : 2,
                    RunPhase.UNDEFINED : 3,
                    RunPhase.WAITING: 4}
# Messages to display on the video
HEAT = "Heatup"
TRAIN = "Training"
EVAL = "Evaluating"
IDLE = "Idle"
UNKWN = "Unknown"
WAITING = "Waiting"
# Allowed label transitions for going from one run phase to another
LABEL_TRANSITION = [[HEAT, TRAIN, EVAL, IDLE, WAITING],
                    [UNKWN, TRAIN, EVAL, IDLE, WAITING],
                    [UNKWN, TRAIN, EVAL, EVAL, WAITING],
                    [HEAT, TRAIN, EVAL, IDLE, WAITING],
                    [HEAT, TRAIN, EVAL, IDLE, WAITING]]


class PhaseObserver(ObserverInterface):
    '''Class that gets notified when the phase changes and publishes the phase to
       a desired topic
    '''
    _phase_pub_node_ = None

    def __init__(self, topic: str, sink: RunPhaseSubject) -> None:
        '''topic - Topic for which to publish the phase '''
        if PhaseObserver._phase_pub_node_ is None:
            PhaseObserver._phase_pub_node_ = rclpy.create_node('MarkovPhaseObserver')
        self._phase_pub_ = self._phase_pub_node_.create_publisher(String, topic, QoSProfile(depth=1))
        self.topic = topic
        self._state_ = None
        sink.register(self)

    def update(self, data: str) -> None:
        try:
            new_state = PHASE_TO_MSG_MAP[data]
            msg = LABEL_TRANSITION[self._state_][new_state] if self._state_ \
                else LABEL_TRANSITION[new_state][new_state]
            self._state_ = new_state
        except KeyError:
            LOG.info('Unknown phase: %s', data)
            msg = UNKWN
            self._state_ = None
        msg_obj = String()
        msg_obj.data = msg
        self._phase_pub_.publish(msg_obj)


def configure_environment_randomizer(light_name_filter=None):
    try:
        get_light_names = ServiceProxyWrapper(GazeboServiceName.GET_LIGHT_NAMES.value, GetLightNames, 
                                            max_retry_attempts=3, timeout_sec=1.0)
        res = get_light_names(GetLightNames.Request())
        
        if res is None:
            LOG.warning("GetLightNames service returned None - no lights will be randomized")
            return
            
        if not res.success:
            LOG.warning("GetLightNames service failed: %s - no lights will be randomized", 
                       res.status_message)
            return
        
        if not res.light_names:
            LOG.info("No lights found in scene for randomization")
            return
            
        LOG.info("Found %d lights in scene: %s", len(res.light_names), res.light_names)
        
        lights_added = 0
        for light_name in res.light_names:
            if light_name_filter and light_name not in light_name_filter:
                LOG.debug("Skipping light '%s' (not in filter)", light_name)
                continue
            RandomizerManager.get_instance().add(LightRandomizer(light_name=light_name))
            lights_added += 1
        
        LOG.info("Added %d light randomizers", lights_added)
        
    except Exception as e:
        LOG.error("Error adding light randomizers: %s", e)
        raise


def signal_robomaker_markov_package_ready():
    """ Create the robomaker_markov_package_ready service on the existing rl_coach node """
    try:
        
        # Use the existing rl_coach node that was passed from evaluation_worker.py
        global _rl_coach_node
        if '_rl_coach_node' in globals() and _rl_coach_node is not None:
            # Create service on the existing rl_coach node
            service = _rl_coach_node.create_service(Empty, '/robomaker_markov_package_ready', handle_robomaker_markov_package_ready)
            LOG.debug('robomaker_markov_package_ready service created on existing rl_coach node')
        else:
            LOG.debug('rl_coach node reference not available')
    except Exception as e:
        LOG.error(f"Failed to create robomaker markov package ready service: {e}")

def handle_robomaker_markov_package_ready(request, response):
    """ This is the handler for responding to the request to check if markov robomaker package
    is up and all the required data is available.

    Args:
        request: Empty service request
        response: Empty service response

    Returns:
        Empty.Response: An empty response stating its ready
    """
    return response

def get_robomaker_profiler_env():
    """ Read robomaker profiler environment """
    is_profiler_on = str2bool(WorldConfig.get_param(ROBOMAKER_IS_PROFILER_ON, False))
    profiler_s3_bucker = WorldConfig.get_param(ROBOMAKER_PROFILER_S3_BUCKET, None)
    profiler_s3_prefix = WorldConfig.get_param(ROBOMAKER_PROFILER_S3_PREFIX, None)
    return is_profiler_on, profiler_s3_bucker, profiler_s3_prefix
