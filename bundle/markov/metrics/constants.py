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

'''This module houses all the constants for the metrics package'''
from enum import Enum, unique
from collections import OrderedDict
import os
import logging
from markov.log_handler.logger import Logger

LOG = Logger(__name__, logging.INFO).get_logger()

class MetricsS3Keys(Enum):
    '''The keys fro the s3 buckets'''
    REGION = 'aws_region'
    METRICS_BUCKET = 'metrics_bucket'
    METRICS_KEY = 'metrics_key'
    ENDPOINT_URL = 'endpoint_url'

class FirehoseStreamKeys(Enum): 
    '''The keys for the Firehose configuration''' 
    REGION = 'aws_region'
    FIREHOSE_DELIVERY_STREAM = 'firehose_delivery_stream'
    FIREHOSE_S3_BUCKET = 'firehose_s3_bucket'
    FIREHOSE_S3_PREFIX = 'firehose_s3_prefix'

class FirehoseUploadFrequency(Enum): 
    STEP_DATA = 'step-data'
    EPISODE_DATA= 'episode-data'

class EvalMetricsKeys(Enum):
    '''The shared metric key for eval metrics'''
    PROGRESS = 'progress'

class StepMetrics(Enum):
    '''The keys for the sim trace metrics'''
    EPISODE = 'episode'
    STEPS = 'steps'
    X = 'X'
    Y = 'Y'
    YAW = 'yaw'
    STEER = 'steer'
    THROTTLE = 'throttle'
    ACTION = 'action'
    REWARD = 'reward'
    DONE = 'done'
    WHEELS_TRACK = 'all_wheels_on_track'
    PROG = 'progress'
    CLS_WAYPNT = 'closest_waypoint'
    TRACK_LEN = 'track_len'
    TIME = 'tstamp'
    EPISODE_STATUS = 'episode_status'
    PAUSE_DURATION = 'pause_duration'
    OBSTACLE_CRASH_COUNTER = "obstacle_crash_counter"

    @classmethod
    def make_default_metric(cls):
        '''Returns the default step metrics dict'''
        step_metrics = OrderedDict()
        for key in cls:
            step_metrics[key.value] = None
        return step_metrics

    @classmethod
    def validate_dict(cls, input_dict):
        '''Throws an exception if a key is missing'''
        for key in cls:
            if input_dict[key.value] is None:
                raise Exception("StepMetrics dict's key({})'s value is None".format(key.value))


class EpisodeStatus(Enum):
    '''The keys for episode status'''
    EPISODE_COMPLETE = 'lap_complete'
    CRASHED = 'crashed'
    OFF_TRACK = 'off_track'
    IN_PROGRESS = 'in_progress'
    IMMOBILIZED = 'immobilized'
    TIME_UP = 'time_up'
    PAUSE = 'pause'
    REVERSED = 'reversed'
    PARK = 'park'
    PREPARE = 'prepare'

    @classmethod
    def get_episode_status(cls, is_done_dict):
        # is_done_dict will have at most one True value or all False
        try:
            episode_status = list(is_done_dict.keys())[list(is_done_dict.values()).index(True)]
            return episode_status
        except ValueError:
            return EpisodeStatus.IN_PROGRESS.value

    @classmethod
    def get_episode_status_label(cls, episode_status):
        if isinstance(episode_status, str):
            return EPISODE_STATUS_LABEL_MAP[episode_status]
        elif isinstance(episode_status, EpisodeStatus):
            return EPISODE_STATUS_LABEL_MAP[episode_status.value]
        else:
            return EPISODE_STATUS_LABEL_MAP[str(episode_status)]


EPISODE_STATUS_LABEL_MAP = {
    EpisodeStatus.EPISODE_COMPLETE.value: 'Lap complete',
    EpisodeStatus.CRASHED.value: 'Crashed',
    EpisodeStatus.OFF_TRACK.value: 'Off track',
    EpisodeStatus.IMMOBILIZED.value: 'Immobilized',
    EpisodeStatus.IN_PROGRESS.value: 'In progress',
    EpisodeStatus.PAUSE.value: 'Pause',
    EpisodeStatus.REVERSED.value: 'Reversed',
    EpisodeStatus.PARK.value: 'Park',
    EpisodeStatus.TIME_UP.value: 'Time Up',
    EpisodeStatus.PREPARE.value: "Prepare"
}

@unique
class Mp4VideoMetrics(Enum):
    '''This enum is used for gathering the video metrics displayed on the mp4'''
    LAP_COUNTER = 'lap_counter'
    COMPLETION_PERCENTAGE = 'completion_percentage'
    RESET_COUNTER = 'reset_counter'
    CRASH_COUNTER = 'crash_counter'
    THROTTLE = 'throttle'
    STEERING = 'steering'
    SPEED = 'speed'
    BEST_LAP_TIME = 'best_lap_time'
    LAST_LAP_TIME = 'last_lap_time'
    TOTAL_EVALUATION_TIME = 'total_evaluation_time'
    DONE = 'done'
    X = 'x'
    Y = 'y'
    OBJECT_LOCATIONS = 'object_locations'
    EPISODE_STATUS = 'episode_status'
    PAUSE_DURATION = 'pause_duration'
    OBSTACLE_RESET_COUNTER = 'obstacle_reset_counter'

    @classmethod
    def get_empty_dict(cls):
        '''Returns dictionary with the string as key values and None's as the values, clients
           are responsible for populating the dict accordingly
        '''
        empty_dict = dict()
        for enum_map in cls._value2member_map_.values():
            empty_dict[enum_map.value] = None
        return empty_dict


class BestModelMetricType(Enum):
    """This enum is used to determine the metric to use when selecting best model"""
    PROGRESS = 'progress'
    REWARD = 'reward'
    LAPTIME = 'laptime'
