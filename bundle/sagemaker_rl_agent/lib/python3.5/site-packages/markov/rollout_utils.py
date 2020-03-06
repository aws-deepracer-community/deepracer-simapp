'''This module should contain common utility methods for the rollout woeker that
   depend on ros, it should not be used in modules imported to the training worker
'''
import logging
import rospy
from std_msgs.msg import String
from markov.agents.utils import RunPhaseSubject
from markov.common import ObserverInterface
from markov.utils import Logger
from rl_coach.core_types import RunPhase

LOG = Logger(__name__, logging.INFO).get_logger()
# Mapping of the phase to the text to display
PHASE_TO_MSG_MAP = {RunPhase.HEATUP : 0,
                    RunPhase.TRAIN : 1,
                    RunPhase.TEST : 2,
                    RunPhase.UNDEFINED : 3}
# Messages to display on the video
HEAT = "Heatup"
TRAIN = "Training"
EVAL = "Evaluating"
IDLE = "Idle"
UNKWN = "Unknown"
# Allowed label transitions for going from one run phase to another
LABEL_TRANSITION = [[HEAT, TRAIN, EVAL, IDLE],
                    [UNKWN, TRAIN, EVAL, IDLE],
                    [UNKWN, TRAIN, EVAL, EVAL],
                    [HEAT, TRAIN, EVAL, IDLE]]

class PhaseObserver(ObserverInterface):
    '''Class that gets notified when the phase changes and publishes the phase to
       a desired topic
    '''
    def __init__(self, topic: str, sink: RunPhaseSubject) -> None:
        '''topic - Topic for which to publish the phase '''
        self._phase_pub_ = rospy.Publisher(topic, String, queue_size=1)
        self._state_ = None
        sink.register(self)

    def update(self, data: str) -> None:
        try:
            new_state = PHASE_TO_MSG_MAP[data]
            msg = LABEL_TRANSITION[self._state_][new_state]  if self._state_ \
                else LABEL_TRANSITION[new_state][new_state]
            self._state_ = new_state
        except KeyError:
            LOG.info('Unknown phase: %s', data)
            msg = UNKWN
            self._state_ = None
        self._phase_pub_.publish(msg)
