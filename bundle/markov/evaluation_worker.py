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

'''This module is responsible for launching evaluation jobs'''
import argparse
import json
import logging
import os
import sys
import time
from threading import Thread
import rclpy
import markov.rollout_utils as rollout_utils

from rl_coach.base_parameters import TaskParameters
from rl_coach.core_types import EnvironmentSteps
from rl_coach.data_stores.data_store import SyncFiles
from markov import utils
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit, simapp_exit_gracefully
from markov.log_handler.constants import (SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                          SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_DONE_EXIT)
from markov.constants import (SIMAPP_VERSION_2, DEFAULT_PARK_POSITION,
                              ROLLOUT_WORKER_PROFILER_PATH)
from markov.agent_ctrl.constants import ConfigParams
from markov.agents.rollout_agent_factory import create_rollout_agent, create_obstacles_agent, create_bot_cars_agent
from markov.agents.utils import RunPhaseSubject
from markov.defaults import reward_function
from markov.log_handler.deepracer_exceptions import GenericRolloutError, GenericRolloutException
from markov.environments.constants import VELOCITY_TOPICS, STEERING_TOPICS, LINK_NAMES
from markov.metrics.s3_metrics import EvalMetrics
from markov.metrics.iteration_data import IterationData
from markov.metrics.constants import FirehoseStreamKeys, FirehoseUploadFrequency, MetricsS3Keys
from markov.s3_boto_data_store import S3BotoDataStore, S3BotoDataStoreParameters
from markov.sagemaker_graph_manager import get_graph_manager
from markov.rollout_utils import (PhaseObserver,
                                  configure_environment_randomizer, get_robomaker_profiler_env,
                                  signal_robomaker_markov_package_ready)
from markov.rclpy_wrappers import ServiceProxyWrapper
from markov.camera_utils import configure_camera
from markov.track_geom.track_data import TrackData
from markov.track_geom.utils import get_start_positions
from markov.track_geom.constants import START_POS_OFFSET, MIN_START_POS_OFFSET, MAX_START_POS_OFFSET
from markov.boto.s3.constants import (MODEL_METADATA_LOCAL_PATH_FORMAT,
                                      MODEL_METADATA_S3_POSTFIX, 
                                      SIMTRACE_EPISODE_EVAL_LOCAL_PATH_FORMAT,
                                      SIMTRACE_EVAL_LOCAL_PATH_FORMAT,
                                      CAMERA_PIP_MP4_LOCAL_PATH_FORMAT,
                                      CAMERA_45DEGREE_LOCAL_PATH_FORMAT,
                                      CAMERA_TOPVIEW_LOCAL_PATH_FORMAT, 
                                      SimtraceEpisodeNames,
                                      SimtraceVideoNames,
                                      ModelMetadataKeys)
from markov.boto.s3.files.model_metadata import ModelMetadata
from markov.boto.s3.files.simtrace_episode import SimtraceEpisodeUpload
from markov.boto.s3.files.simtrace_video import SimtraceVideo
from markov.boto.s3.files.checkpoint import Checkpoint
from markov.boto.s3.utils import get_s3_key
from markov.reset.constants import RaceType
from markov.world_config import WorldConfig

from std_srvs.srv import Empty

enable_episode_simtrace = utils.str2bool(WorldConfig.get_param("ENABLE_EPISODE_SIMTRACE", False)) # feature flag
enable_firehose_upload = utils.str2bool(WorldConfig.get_param("ENABLE_FIREHOSE_UPLOAD", False)) # feature flag

logger = Logger(__name__, logging.INFO).get_logger()

MIN_RESET_COUNT = 2 #Reduced reset limit to avoid eval without box of doom

## Suppress unnecessary logs from these modules
logging.getLogger('rl_coach').setLevel(logging.ERROR)
logging.getLogger('tensorflow').setLevel(logging.ERROR)

IS_PROFILER_ON, PROFILER_S3_BUCKET, PROFILER_S3_PREFIX = get_robomaker_profiler_env()


def evaluation_worker(graph_manager, number_of_trials, task_parameters, simtrace_video_s3_writers, is_continuous,
                      park_positions, race_type, pause_physics, unpause_physics):
    """ Evaluation worker function

    Arguments:
        graph_manager(MultiAgentGraphManager): Graph manager of multiagent graph manager
        number_of_trials(int): Number of trails you want to run the evaluation
        task_parameters(TaskParameters): Information of the checkpoint, gpu/cpu,
            framework etc of rlcoach
        simtrace_video_s3_writers(list): Information to upload to the S3 bucket all the simtrace and mp4
        is_continuous(bool): The termination condition for the car
        park_positions(list of tuple): list of (x, y) for cars to park at
        race_type (str): race type
    """
    # Collect profiler information only IS_PROFILER_ON is true
    with utils.Profiler(s3_bucket=PROFILER_S3_BUCKET, s3_prefix=PROFILER_S3_PREFIX,
                        output_local_path=ROLLOUT_WORKER_PROFILER_PATH, enable_profiling=IS_PROFILER_ON):
        subscribe_to_save_mp4_topic, unsubscribe_from_save_mp4_topic = list(), list()
        subscribe_to_save_mp4, unsubscribe_from_save_mp4 = list(), list()
        for agent_param in graph_manager.agents_params:
            racecar_name = 'racecar' if len(agent_param.name.split("_")) == 1 \
                                     else "racecar_{}".format(agent_param.name.split("_")[1])
            subscribe_to_save_mp4_topic.append("/{}/save_mp4/subscribe_to_save_mp4".format(racecar_name))
            unsubscribe_from_save_mp4_topic.append("/{}/save_mp4/unsubscribe_from_save_mp4".format(racecar_name))
        
        graph_manager.data_store.wait_for_checkpoints()
        graph_manager.data_store.modify_checkpoint_variables()

        # Make the clients that will allow us to pause and unpause the physics
        pause_physics = ServiceProxyWrapper('/deepracer/pause_physics_dr')
        unpause_physics = ServiceProxyWrapper('/deepracer/unpause_physics_dr')

        for mp4_sub, mp4_unsub in zip(subscribe_to_save_mp4_topic, unsubscribe_from_save_mp4_topic):
            subscribe_to_save_mp4.append(ServiceProxyWrapper(mp4_sub))
            unsubscribe_from_save_mp4.append(Thread(target=ServiceProxyWrapper(mp4_unsub, timeout_sec=60.0),
                                                    args=(Empty.Request(), )))

        graph_manager.create_graph(task_parameters=task_parameters, stop_physics=pause_physics,
                                   start_physics=unpause_physics, empty_service_call=Empty.Request)
        unpause_physics(Empty.Request())

        is_save_mp4_enabled = WorldConfig.get_param('MP4_S3_BUCKET', None)
        if is_save_mp4_enabled:
            
            for i, subscribe_mp4 in enumerate(subscribe_to_save_mp4):
                try:
                    # Add timeout and retry logic
                    max_retries = 3
                    retry_delay = 2.0
                    
                    for retry in range(max_retries):
                        try:
                            subscribe_mp4(Empty.Request())
                            break
                        except Exception as service_error:
                            if retry < max_retries - 1:
                                time.sleep(retry_delay)
                            else:
                                logger.debug("Max retries reached")
                                
                except Exception as e:
                    logger.error(f"Error in evaluation worker: {e}")
            

        configure_environment_randomizer()
        track_data = TrackData.get_instance()

        # Before each evaluation episode (single lap for non-continuous race and complete race for
        # continuous race), a new copy of park_positions needs to be loaded into track_data because
        # a park position will be pop from park_positions when a racer car need to be parked.
        if is_continuous:
            track_data.park_positions = park_positions
            graph_manager.evaluate(EnvironmentSteps(1))
        else:
            for trial_idx in range(number_of_trials):
                track_data.park_positions = park_positions
                graph_manager.evaluate(EnvironmentSteps(1))
        if is_save_mp4_enabled:
            for unsubscribe_mp4 in unsubscribe_from_save_mp4:
                unsubscribe_mp4.start()
            for unsubscribe_mp4 in unsubscribe_from_save_mp4:
                unsubscribe_mp4.join()
        # upload simtrace and mp4 into s3 bucket
        for s3_writer in simtrace_video_s3_writers:
            s3_writer.persist(utils.get_s3_kms_extra_args())
        time.sleep(1)
        pause_physics(Empty.Request())
    handle_job_completion()

def handle_job_completion():
    logger.info("Evaluation job complete")
    simapp_exit_gracefully(simapp_exit=SIMAPP_DONE_EXIT, name='Eval Worker')

def main():
    """ Main function for evaluation worker """
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--preset',
                        help="(string) Name of a preset to run \
                             (class name from the 'presets' directory.)",
                        type=str,
                        required=False)
    parser.add_argument('--s3_bucket',
                        help='list(string) S3 bucket',
                        type=str,
                        nargs='+',
                        default=WorldConfig.get_param("MODEL_S3_BUCKET", ["gsaur-test"]))
    parser.add_argument('--s3_prefix',
                        help='list(string) S3 prefix',
                        type=str,
                        nargs='+',
                        default=WorldConfig.get_param("MODEL_S3_PREFIX", ["sagemaker"]))
    parser.add_argument('--s3_endpoint_url',
                        help='(string) S3 endpoint URL',
                        type=str,
                        default=WorldConfig.get_param("S3_ENDPOINT_URL", None))                        
    parser.add_argument('--aws_region',
                        help='(string) AWS region',
                        type=str,
                        default=WorldConfig.get_param("AWS_REGION", "us-east-1"))
    parser.add_argument('--number_of_trials',
                        help='(integer) Number of trials',
                        type=int,
                        default=int(WorldConfig.get_param("NUMBER_OF_TRIALS", 10)))
    parser.add_argument('-c', '--local_model_directory',
                        help='(string) Path to a folder containing a checkpoint \
                             to restore the model from.',
                        type=str,
                        default='./checkpoint')
    parser.add_argument('--number_of_resets',
                        help='(integer) Number of resets',
                        type=int,
                        default=int(WorldConfig.get_param("NUMBER_OF_RESETS", 0)))
    parser.add_argument('--penalty_seconds',
                        help='(float) penalty second',
                        type=float,
                        default=float(WorldConfig.get_param("PENALTY_SECONDS", 2.0)))
    parser.add_argument('--job_type',
                        help='(string) job type',
                        type=str,
                        default=WorldConfig.get_param("JOB_TYPE", "EVALUATION"))
    parser.add_argument('--is_continuous',
                        help='(boolean) is continous after lap completion',
                        type=bool,
                        default=utils.str2bool(WorldConfig.get_param("IS_CONTINUOUS", False)))
    parser.add_argument('--race_type',
                        help='(string) Race type',
                        type=str,
                        default=WorldConfig.get_param("RACE_TYPE", "TIME_TRIAL"))
    parser.add_argument('--off_track_penalty',
                        help='(float) off track penalty second',
                        type=float,
                        default=float(WorldConfig.get_param("OFF_TRACK_PENALTY", 2.0)))
    parser.add_argument('--collision_penalty',
                        help='(float) collision penalty second',
                        type=float,
                        default=float(WorldConfig.get_param("COLLISION_PENALTY", 5.0)))
    parser.add_argument('--round_robin_advance_dist',
                        help='(float) round robin distance 0-1',
                        type=float,
                        default=float(WorldConfig.get_param("ROUND_ROBIN_ADVANCE_DIST", 0.05)))
    parser.add_argument('--start_position_offset',
                        help='(float) offset start 0-1',
                        type=float,
                        default=float(WorldConfig.get_param("START_POSITION_OFFSET", 0.0)))
    parser.add_argument('--eval_checkpoint',
                        help='(string) Choose which checkpoint to use (best | last)',
                        type=str,
                        default=WorldConfig.get_param("EVAL_CHECKPOINT", "best"))

    args = parser.parse_args()
    arg_s3_bucket = args.s3_bucket
    arg_s3_prefix = args.s3_prefix
    logger.info("S3 bucket: %s \n S3 prefix: %s \n S3 endpoint URL: %s", args.s3_bucket, args.s3_prefix, args.s3_endpoint_url)

    metrics_s3_buckets = WorldConfig.get_param('METRICS_S3_BUCKET', 'default_bucket')
    metrics_s3_object_keys = WorldConfig.get_param('METRICS_S3_OBJECT_KEY', 'metrics.json')
    start_pos_offset = max(min(float(WorldConfig.get_param("START_POS_OFFSET", START_POS_OFFSET)), MAX_START_POS_OFFSET),
                           MIN_START_POS_OFFSET)

    arg_s3_bucket, arg_s3_prefix = utils.force_list(arg_s3_bucket), utils.force_list(arg_s3_prefix)
    metrics_s3_buckets = utils.force_list(metrics_s3_buckets)
    metrics_s3_object_keys = utils.force_list(metrics_s3_object_keys)

    validate_list = [arg_s3_bucket, arg_s3_prefix, metrics_s3_buckets, metrics_s3_object_keys]

    simtrace_s3_bucket = WorldConfig.get_param('SIMTRACE_S3_BUCKET', None)
    mp4_s3_bucket = WorldConfig.get_param('MP4_S3_BUCKET', None)
    if simtrace_s3_bucket: 
        # OLD ROS1 style: simtrace_s3_object_prefix = WorldConfig.get_param('SIMTRACE_S3_PREFIX') 
        # FIXED ROS2: Add required default_value parameter
        simtrace_s3_object_prefix = WorldConfig.get_param('SIMTRACE_S3_PREFIX', '') 
        simtrace_s3_bucket = utils.force_list(simtrace_s3_bucket) 
        simtrace_s3_object_prefix = utils.force_list(simtrace_s3_object_prefix) 
        validate_list.extend([simtrace_s3_bucket, simtrace_s3_object_prefix]) 
        # Only enable this when the feature flag is on
        if enable_episode_simtrace: 
            # OLD ROS1 style: simtrace_episode_s3_object_prefix = WorldConfig.get_param('SIMTRACE_EPISODE_S3_PREFIX') 
            # FIXED ROS2: Add required default_value parameter
            simtrace_episode_s3_object_prefix = WorldConfig.get_param('SIMTRACE_EPISODE_S3_PREFIX', '') 
            simtrace_episode_s3_object_prefix = utils.force_list(simtrace_episode_s3_object_prefix)
            validate_list.append(simtrace_episode_s3_object_prefix)
    if mp4_s3_bucket:
        # OLD ROS1 style: mp4_s3_object_prefix = WorldConfig.get_param('MP4_S3_OBJECT_PREFIX')
        # FIXED ROS2: Add required default_value parameter
        mp4_s3_object_prefix = WorldConfig.get_param('MP4_S3_OBJECT_PREFIX', '')
        mp4_s3_bucket = utils.force_list(mp4_s3_bucket)
        mp4_s3_object_prefix = utils.force_list(mp4_s3_object_prefix)
        validate_list.extend([mp4_s3_bucket, mp4_s3_object_prefix])

    if enable_firehose_upload:
        firehose_s3_bucket = utils.force_list(WorldConfig.get_param('FIREHOSE_S3_BUCKET', None))
        if firehose_s3_bucket:
            # ROS2: Add required default_value parameter
            firehose_metrics_stream_names = utils.force_list(WorldConfig.get_param('FIREHOSE_METRICS_STREAM_NAME', ''))
            firehose_simtrace_stream_names = utils.force_list(WorldConfig.get_param('FIREHOSE_SIMTRACE_STREAM_NAME', ''))
            firehose_metrics_s3_prefixes = utils.force_list(WorldConfig.get_param('FIREHOSE_METRICS_S3_PREFIX', ''))
            firehose_simtrace_s3_prefixes = utils.force_list(WorldConfig.get_param('FIREHOSE_SIMTRACE_S3_PREFIX', ''))
    
            validate_list.extend([firehose_metrics_stream_names, firehose_simtrace_stream_names, firehose_s3_bucket,
                                firehose_metrics_s3_prefixes, firehose_simtrace_s3_prefixes])

    if not all([lambda x: len(x) == len(validate_list[0]), validate_list]):
        log_and_exit("Eval worker error: Incorrect arguments passed: {}"
                         .format(validate_list),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500,
                     name="Eval worker")
    if args.number_of_resets != 0 and args.number_of_resets < MIN_RESET_COUNT:
        raise GenericRolloutException("number of resets is less than {}".format(MIN_RESET_COUNT))

    # Instantiate Cameras
    camera_main_enable = utils.str2bool(WorldConfig.get_param("CAMERA_MAIN_ENABLE", "True"))
    camera_sub_enable = utils.str2bool(WorldConfig.get_param("CAMERA_SUB_ENABLE", "True"))

    # Wait for car spawn delay (racecar_control_kinematics.launch.py has 30s TimerAction before spawning controllers)
    time.sleep(30)

    if camera_main_enable:
        if len(arg_s3_bucket) == 1:
            configure_camera(namespaces=['racecar'])
        else:
            configure_camera(namespaces=[
                'racecar_{}'.format(str(agent_index)) for agent_index in range(len(arg_s3_bucket))])

    agent_list = list()
    s3_bucket_dict = dict()
    s3_prefix_dict = dict()
    checkpoint_dict = dict()
    simtrace_video_s3_writers = []
    simtrace_episode_s3_writer = None
    start_positions = get_start_positions(len(arg_s3_bucket), start_pos_offset)
    done_condition = utils.str_to_done_condition(WorldConfig.get_param("DONE_CONDITION", any))
    park_positions = utils.pos_2d_str_to_list(WorldConfig.get_param("PARK_POSITIONS", []))
    # if not pass in park positions for all done condition case, use default
    if not park_positions:
        park_positions = [DEFAULT_PARK_POSITION for _ in arg_s3_bucket]
    for agent_index, _ in enumerate(arg_s3_bucket):
        agent_name = 'agent' if len(arg_s3_bucket) == 1 else 'agent_{}'.format(str(agent_index))
        racecar_name = 'racecar' if len(arg_s3_bucket) == 1 else 'racecar_{}'.format(str(agent_index))
        s3_bucket_dict[agent_name] = arg_s3_bucket[agent_index]
        s3_prefix_dict[agent_name] = arg_s3_prefix[agent_index]

        # download model metadata
        model_metadata = ModelMetadata(bucket=arg_s3_bucket[agent_index],
                                       s3_key=get_s3_key(arg_s3_prefix[agent_index], MODEL_METADATA_S3_POSTFIX),
                                       region_name=args.aws_region,
                                       s3_endpoint_url=args.s3_endpoint_url,
                                       local_path=MODEL_METADATA_LOCAL_PATH_FORMAT.format(agent_name))
        model_metadata_info = model_metadata.get_model_metadata_info()
        version = model_metadata_info[ModelMetadataKeys.VERSION.value]


        # checkpoint s3 instance
        checkpoint = Checkpoint(bucket=arg_s3_bucket[agent_index],
                                s3_prefix=arg_s3_prefix[agent_index],
                                region_name=args.aws_region,
                                s3_endpoint_url=args.s3_endpoint_url,
                                agent_name=agent_name,
                                checkpoint_dir=args.local_model_directory)
        # make coach checkpoint compatible
        if version < SIMAPP_VERSION_2 and not checkpoint.rl_coach_checkpoint.is_compatible():
            checkpoint.rl_coach_checkpoint.make_compatible(checkpoint.syncfile_ready)

        # Get the correct checkpoint
        if args.eval_checkpoint.lower() == "best":
            # get best model checkpoint string
            model_checkpoint_name = checkpoint.deepracer_checkpoint_json.get_deepracer_best_checkpoint()
        else:
            # get the last model checkpoint string
            model_checkpoint_name = checkpoint.deepracer_checkpoint_json.get_deepracer_last_checkpoint()

        # Select the best checkpoint model by uploading rl coach .coach_checkpoint file
        checkpoint.rl_coach_checkpoint.update(
            model_checkpoint_name=model_checkpoint_name,
            s3_kms_extra_args=utils.get_s3_kms_extra_args())

        checkpoint_dict[agent_name] = checkpoint

        agent_config = {
            'model_metadata': model_metadata,
            ConfigParams.CAR_CTRL_CONFIG.value: {
                ConfigParams.LINK_NAME_LIST.value: [
                    link_name.replace('racecar', racecar_name) for link_name in LINK_NAMES],
                ConfigParams.VELOCITY_LIST.value: [
                    velocity_topic.replace('racecar', racecar_name) for velocity_topic in VELOCITY_TOPICS],
                ConfigParams.STEERING_LIST.value: [
                    steering_topic.replace('racecar', racecar_name) for steering_topic in STEERING_TOPICS],
                ConfigParams.CHANGE_START.value: utils.str2bool(WorldConfig.get_param('CHANGE_START_POSITION', False)),
                ConfigParams.ALT_DIR.value: utils.str2bool(WorldConfig.get_param('ALTERNATE_DRIVING_DIRECTION', False)),
                ConfigParams.MODEL_METADATA.value: model_metadata,
                ConfigParams.REWARD.value: reward_function,
                ConfigParams.AGENT_NAME.value: racecar_name,
                ConfigParams.VERSION.value: version,
                ConfigParams.NUMBER_OF_RESETS.value: args.number_of_resets,
                ConfigParams.PENALTY_SECONDS.value: args.penalty_seconds,
                ConfigParams.NUMBER_OF_TRIALS.value: args.number_of_trials,
                ConfigParams.IS_CONTINUOUS.value: args.is_continuous,
                ConfigParams.RACE_TYPE.value: args.race_type,
                ConfigParams.COLLISION_PENALTY.value: args.collision_penalty,
                ConfigParams.OFF_TRACK_PENALTY.value: args.off_track_penalty,
                ConfigParams.START_POSITION.value: start_positions[agent_index],
                ConfigParams.DONE_CONDITION.value: done_condition,
                ConfigParams.ROUND_ROBIN_ADVANCE_DIST.value: args.round_robin_advance_dist,
                ConfigParams.START_POSITION_OFFSET.value: args.start_position_offset}}

        metrics_s3_config = {MetricsS3Keys.METRICS_BUCKET.value: metrics_s3_buckets[agent_index],
                             MetricsS3Keys.METRICS_KEY.value: metrics_s3_object_keys[agent_index],
                             MetricsS3Keys.ENDPOINT_URL.value: WorldConfig.get_param('S3_ENDPOINT_URL', None),
                             # Replaced rospy.get_param('AWS_REGION') to be equal to the argument being passed
                             # or default argument set
                             MetricsS3Keys.REGION.value: args.aws_region}

        firehose_metrics_config = None
        firehose_simtrace_config= None
        # Only enable this when the feature flag is on
        if enable_firehose_upload:
            firehose_upload_frequency = WorldConfig.get_param('FIREHOSE_UPLOAD_FREQUENCY')
            if firehose_upload_frequency not in FirehoseUploadFrequency._value2member_map_:
                log_and_exit("Eval worker error: Invalid FIREHOSE_UPLOAD_FREQUENCY: {}. "
                             "Expected one of: {}".format(firehose_upload_frequency, ", "
                                                          .join([e.value for e in FirehoseUploadFrequency])),
                             SIMAPP_SIMULATION_WORKER_EXCEPTION,
                             SIMAPP_EVENT_ERROR_CODE_500,
                             name="Eval Worker")
                
            firehose_metrics_config = {FirehoseStreamKeys.FIREHOSE_DELIVERY_STREAM.value: firehose_metrics_stream_names[agent_index],
                                       FirehoseStreamKeys.FIREHOSE_S3_BUCKET.value: firehose_s3_bucket[agent_index],
                                       FirehoseStreamKeys.FIREHOSE_S3_PREFIX.value: firehose_metrics_s3_prefixes[agent_index],
                                       FirehoseStreamKeys.REGION.value: args.aws_region} 
            firehose_simtrace_config = {FirehoseStreamKeys.FIREHOSE_DELIVERY_STREAM.value: firehose_simtrace_stream_names[agent_index],
                                        FirehoseStreamKeys.FIREHOSE_S3_BUCKET.value: firehose_s3_bucket[agent_index],
                                        FirehoseStreamKeys.FIREHOSE_S3_PREFIX.value: firehose_simtrace_s3_prefixes[agent_index],
                                        FirehoseStreamKeys.REGION.value: args.aws_region}
                
        aws_region = WorldConfig.get_param('AWS_REGION', args.aws_region)
        if simtrace_s3_bucket:
            # Only enable this when the feature flag is on
            if enable_episode_simtrace:
                simtrace_episode_s3_writer = SimtraceEpisodeUpload(upload_type=SimtraceEpisodeNames.SIMTRACE_EVAL.value, 
                                                                   bucket=simtrace_s3_bucket[agent_index], 
                                                                   s3_prefix=simtrace_episode_s3_object_prefix[agent_index], 
                                                                   region_name=aws_region, 
                                                                   s3_endpoint_url=args.s3_endpoint_url,
                                                                   local_path=SIMTRACE_EPISODE_EVAL_LOCAL_PATH_FORMAT.format(agent_name))
            simtrace_video_s3_writers.append(
                SimtraceVideo(upload_type=SimtraceVideoNames.SIMTRACE_EVAL.value,
                              bucket=simtrace_s3_bucket[agent_index],
                              s3_prefix=simtrace_s3_object_prefix[agent_index],
                              region_name=aws_region,
                              s3_endpoint_url=args.s3_endpoint_url,
                              local_path=SIMTRACE_EVAL_LOCAL_PATH_FORMAT.format(agent_name)))
        if mp4_s3_bucket:
            simtrace_video_s3_writers.extend([
                SimtraceVideo(upload_type=SimtraceVideoNames.PIP.value,
                              bucket=mp4_s3_bucket[agent_index],
                              s3_prefix=mp4_s3_object_prefix[agent_index],
                              region_name=aws_region,
                              s3_endpoint_url=args.s3_endpoint_url,
                              local_path=CAMERA_PIP_MP4_LOCAL_PATH_FORMAT.format(agent_name)),
                SimtraceVideo(upload_type=SimtraceVideoNames.DEGREE45.value,
                              bucket=mp4_s3_bucket[agent_index],
                              s3_prefix=mp4_s3_object_prefix[agent_index],
                              region_name=aws_region,
                              s3_endpoint_url=args.s3_endpoint_url,
                              local_path=CAMERA_45DEGREE_LOCAL_PATH_FORMAT.format(agent_name)),
                SimtraceVideo(upload_type=SimtraceVideoNames.TOPVIEW.value,
                              bucket=mp4_s3_bucket[agent_index],
                              s3_prefix=mp4_s3_object_prefix[agent_index],
                              region_name=aws_region,
                              s3_endpoint_url=args.s3_endpoint_url,
                              local_path=CAMERA_TOPVIEW_LOCAL_PATH_FORMAT.format(agent_name))])

        run_phase_subject = RunPhaseSubject()
        agent_list.append(create_rollout_agent(agent_config, EvalMetrics(agent_name, metrics_s3_config,
                                                                         args.is_continuous,
                                                                         simtrace_episode_s3_writer,
                                                                         firehose_metrics_config,
                                                                         firehose_simtrace_config),
                                               run_phase_subject))
    agent_list.append(create_obstacles_agent())
    agent_list.append(create_bot_cars_agent())

    # FIXED: ROS service to indicate all the robomaker markov packages are ready for consumption
    # This matches ROS1 pattern - called EARLY after agents created, not at the end
    signal_robomaker_markov_package_ready()

    PhaseObserver('/agent/training_phase', run_phase_subject)
    enable_domain_randomization = utils.str2bool(WorldConfig.get_param('ENABLE_DOMAIN_RANDOMIZATION', False))

    sm_hyperparams_dict = {}

    # Make the clients that will allow us to pause and unpause the physics
    pause_physics = ServiceProxyWrapper('/deepracer/pause_physics_dr')
    unpause_physics = ServiceProxyWrapper('/deepracer/unpause_physics_dr')

    graph_manager, _ = get_graph_manager(hp_dict=sm_hyperparams_dict, agent_list=agent_list,
                                         run_phase_subject=run_phase_subject,
                                         enable_domain_randomization=enable_domain_randomization,
                                         done_condition=done_condition,
                                         pause_physics=pause_physics,
                                         unpause_physics=unpause_physics)

    ds_params_instance = S3BotoDataStoreParameters(checkpoint_dict=checkpoint_dict)

    graph_manager.data_store = S3BotoDataStore(params=ds_params_instance,
                                               graph_manager=graph_manager,
                                               ignore_lock=True)
    graph_manager.env_params.seed = 0

    task_parameters = TaskParameters()
    task_parameters.checkpoint_restore_path = args.local_model_directory

    
    evaluation_worker(
        graph_manager=graph_manager,
        number_of_trials=args.number_of_trials,
        task_parameters=task_parameters,
        simtrace_video_s3_writers=simtrace_video_s3_writers,
        is_continuous=args.is_continuous,
        park_positions=park_positions,
        race_type=args.race_type,
        pause_physics=pause_physics,
        unpause_physics=unpause_physics
    )

if __name__ == '__main__':
    try:
        rclpy.init()
        node = rclpy.create_node('rl_coach')
        
        # Create the robomaker_markov_package_ready service immediately
        rollout_utils._rl_coach_node = node  # Store node reference globally
        signal_robomaker_markov_package_ready()
        
        try:
            main()
        except Exception as e:
            raise
        
        # Successful completion - log and exit
        logger.info("Markov evaluation worker completed successfully")
        if rclpy.ok():
            logger.info("Shutting down ROS...")
            rclpy.shutdown()
        sys.exit(0)
    except ValueError as err:
        if utils.is_user_error(err):
            log_and_exit("User modified model/model_metadata: {}".format(err),
                         SIMAPP_SIMULATION_WORKER_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500,
                         name="Eval Worker")
        else:
            log_and_exit("Eval worker value error: {}".format(err),
                         SIMAPP_SIMULATION_WORKER_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500,
                         name="Eval Worker")
    except GenericRolloutError as ex:
        ex.log_except_and_exit()
    except GenericRolloutException as ex:
        ex.log_except_and_exit()
    except Exception as ex:
        log_and_exit("Eval worker error: {}".format(ex),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500,
                     name="Eval Worker")
