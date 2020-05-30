#!/usr/bin/env python3

"""script to download yaml param from S3 bucket to local directory and start tournament ROS launch

Example:
    $ python tournament_node.py s3_region, s3_bucket, s3_prefix, s3_yaml_name
"""

from collections import deque
import copy
import json
import logging
import os
import pickle
import sys
import yaml
import shutil
import subprocess
import uuid
import boto3
import time
import rospy

from markov import utils_parse_model_metadata
from markov.rollout_constants import BodyShellType
from markov.utils import (force_list, restart_simulation_job,
                          cancel_simulation_job, get_boto_config, get_s3_kms_extra_args)
from markov.architecture.constants import Input
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_WORKER_EXCEPTION)

from download_params_and_roslaunch_agent import get_yaml_dict, get_yaml_values, F1_SHELL_USERS_LIST

logger = Logger(__name__, logging.INFO).get_logger()
# Amount of time to wait to guarantee that RoboMaker's network configuration is ready.
WAIT_FOR_ROBOMAKER_TIME = 10

RACE_CAR_COLORS = ["Purple", "Orange"]

CAR_COLOR_YAML_KEY = "CAR_COLOR"
BODY_SHELL_TYPE_YAML_KEY = "BODY_SHELL_TYPE"
MODEL_S3_BUCKET_YAML_KEY = "MODEL_S3_BUCKET"
MODEL_S3_PREFIX_YAML_KEY = "MODEL_S3_PREFIX"
MODEL_METADATA_FILE_S3_YAML_KEY = "MODEL_METADATA_FILE_S3_KEY"
METRICS_S3_BUCKET_YAML_KEY = "METRICS_S3_BUCKET"
METRICS_S3_OBJECT_KEY_YAML_KEY = "METRICS_S3_OBJECT_KEY"
METRICS_S3_PREFIX_YAML_KEY = "METRICS_S3_PREFIX"
SIMTRACE_S3_BUCKET_YAML_KEY = "SIMTRACE_S3_BUCKET"
SIMTRACE_S3_PREFIX_YAML_KEY = "SIMTRACE_S3_PREFIX"
MP4_S3_BUCKET_YAML_KEY = "MP4_S3_BUCKET"
MP4_S3_PREFIX_YAML_KEY = "MP4_S3_OBJECT_PREFIX"
DISPLAY_NAME_YAML_KEY = "DISPLAY_NAME"
VIDEO_JOB_TYPE_YAML_KEY = "VIDEO_JOB_TYPE"
RACER_NAME_YAML_KEY = "RACER_NAME"
LEADERBOARD_TYPE_YAML_KEY = "LEADERBOARD_TYPE"
LEADERBOARD_NAME_YAML_KEY = "LEADERBOARD_NAME"

S3_DOWNLOAD_ERROR_MSG_FORMAT = "S3 download failed. Re-try count: {0}/{1}: [S3_Bucket: {2}, S3_Key: {3}]: {4}"


def run_cmd(cmd_args, change_working_directory="./", shell=False,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            executable=None):
    """
    Function used to execute the shell commands

    return (returncode, stdout, stderr)
    : returncode: int - This contains the 0/1 based on whether command
    : stdout: bytes - stdout from the command execution
    : stderr: bytes - stderr from the command execution

    :param cmd_args: list - This is the list of commands to be appended in case
        of shell=True, else executed as pipes.
    :param change_working_directory: string - This is to execute the command in different
        directory
    :param shell: bool - This is to say if the command to be executed as shell or not
    :param stdout: int - stdout stream target
    :param stderr: int - stderr stream target
    """
    cmd = " ".join(map(str, cmd_args))
    process = subprocess.Popen(
        cmd if shell else cmd_args,
        cwd=change_working_directory,
        shell=shell,
        stdout=stdout,
        stderr=stderr,
        executable=executable
    )
    stdout, stderr = process.communicate()
    return process.returncode, stdout, stderr


def generate_race_yaml(yaml_dict, car1, car2, race_idx):
    """Generate racer yaml configs

    Arguments:
        yaml_dict {dict} -- original yaml config
        car1 {tuple} -- tuple containing car1 information
        car2 {tuple} -- tuple containing car2 information
        race_idx {int} -- race index

    """
    (car1_model_s3_bucket, car1_s3_prefix, car1_model_metadata,
     car1_metrics_bucket, car1_metrics_s3_prefix,
     car1_simtrace_bucket, car1_simtrace_prefix,
     car1_mp4_bucket, car1_mp4_prefix,
     car1_display_name, car1_racer_name,
     car1_body_shell_type) = car1
    (car2_model_s3_bucket, car2_s3_prefix, car2_model_metadata,
     car2_metrics_bucket, car2_metrics_s3_prefix,
     car2_simtrace_bucket, car2_simtrace_prefix,
     car2_mp4_bucket, car2_mp4_prefix,
     car2_display_name, car2_racer_name,
     car2_body_shell_type) = car2

    race_yaml_dict = copy.deepcopy(yaml_dict)

    race_yaml_dict[CAR_COLOR_YAML_KEY] = RACE_CAR_COLORS
    race_yaml_dict[MODEL_S3_BUCKET_YAML_KEY] = [car1_model_s3_bucket, car2_model_s3_bucket]
    race_yaml_dict[MODEL_S3_PREFIX_YAML_KEY] = [car1_s3_prefix, car2_s3_prefix]
    race_yaml_dict[MODEL_METADATA_FILE_S3_YAML_KEY] = [car1_model_metadata, car2_model_metadata]

    race_yaml_dict[METRICS_S3_BUCKET_YAML_KEY] = [car1_metrics_bucket, car2_metrics_bucket]
    race_yaml_dict[METRICS_S3_OBJECT_KEY_YAML_KEY] = [os.path.join(car1_metrics_s3_prefix,
                                                                   str(uuid.uuid4()),
                                                                   'evaluation_metrics.json'),
                                                      os.path.join(car2_metrics_s3_prefix,
                                                                   str(uuid.uuid4()),
                                                                   'evaluation_metrics.json')]
    race_yaml_dict[SIMTRACE_S3_BUCKET_YAML_KEY] = [car1_simtrace_bucket, car2_simtrace_bucket]
    race_yaml_dict[SIMTRACE_S3_PREFIX_YAML_KEY] = [os.path.join(car1_simtrace_prefix, str(uuid.uuid4())),
                                                   os.path.join(car2_simtrace_prefix, str(uuid.uuid4()))]
    race_yaml_dict[MP4_S3_BUCKET_YAML_KEY] = [car1_mp4_bucket, car2_mp4_bucket]
    race_yaml_dict[MP4_S3_PREFIX_YAML_KEY] = [os.path.join(car1_mp4_prefix, str(uuid.uuid4())),
                                              os.path.join(car2_mp4_prefix, str(uuid.uuid4()))]
    race_yaml_dict[DISPLAY_NAME_YAML_KEY] = [car1_display_name, car2_display_name]
    # TODO - Deprecate the DISPLAY_NAME once cloud team pushes the RACER_NAME yaml
    race_yaml_dict[RACER_NAME_YAML_KEY] = [car1_racer_name, car2_racer_name]
    race_yaml_dict[BODY_SHELL_TYPE_YAML_KEY] = [car1_body_shell_type, car2_body_shell_type]
    return race_yaml_dict


def main():
    """ Main function for tournament"""
    try:
        # parse argument
        s3_region = sys.argv[1]
        s3_bucket = sys.argv[2]
        s3_prefix = sys.argv[3]
        s3_yaml_name = sys.argv[4]

        # create boto3 session/client and download yaml/json file
        session = boto3.session.Session()
        s3_endpoint_url = os.environ.get("S3_ENDPOINT_URL", None)
        s3_client = session.client('s3', region_name=s3_region, endpoint_url=s3_endpoint_url, config=get_boto_config())

        yaml_key = os.path.normpath(os.path.join(s3_prefix, s3_yaml_name))
        local_yaml_path = os.path.abspath(os.path.join(os.getcwd(), s3_yaml_name))
        try:
            s3_client.download_file(Bucket=s3_bucket, Key=yaml_key, Filename=local_yaml_path)
        except Exception as e:
            log_and_exit("Failed to download yaml file: s3_bucket: {}, yaml_key: {}, {}"
                             .format(s3_bucket, yaml_key, e),
                         SIMAPP_SIMULATION_WORKER_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)

        # Intermediate tournament files
        queue_pickle_name = 'tournament_candidate_queue.pkl'
        queue_pickle_s3_key = os.path.normpath(os.path.join(s3_prefix, queue_pickle_name))
        local_queue_pickle_path = os.path.abspath(os.path.join(os.getcwd(), queue_pickle_name))

        report_pickle_name = 'tournament_report.pkl'
        report_pickle_s3_key = os.path.normpath(os.path.join(s3_prefix, report_pickle_name))
        local_report_pickle_path = os.path.abspath(os.path.join(os.getcwd(), report_pickle_name))

        final_report_name = 'tournament_report.json'
        final_report_s3_key = os.path.normpath(os.path.join(s3_prefix, final_report_name))

        try:
            s3_client.download_file(Bucket=s3_bucket,
                                    Key=queue_pickle_s3_key,
                                    Filename=local_queue_pickle_path)
            s3_client.download_file(Bucket=s3_bucket,
                                    Key=report_pickle_s3_key,
                                    Filename=local_report_pickle_path)
        except:
            pass

        # Get values passed in yaml files. Default values are for backward compatibility and for single racecar racing
        yaml_dict = get_yaml_dict(local_yaml_path)

        # Forcing the yaml parameter to list
        # TODO: Deprecate the DISPLAY_NAME and use only the RACER_NAME after cloud pushes this YAML parameter
        force_list_params = [MODEL_S3_BUCKET_YAML_KEY, MODEL_S3_PREFIX_YAML_KEY, MODEL_METADATA_FILE_S3_YAML_KEY,
                             METRICS_S3_BUCKET_YAML_KEY, METRICS_S3_PREFIX_YAML_KEY,
                             SIMTRACE_S3_BUCKET_YAML_KEY, SIMTRACE_S3_PREFIX_YAML_KEY,
                             MP4_S3_BUCKET_YAML_KEY, MP4_S3_PREFIX_YAML_KEY,
                             DISPLAY_NAME_YAML_KEY, RACER_NAME_YAML_KEY,
                             BODY_SHELL_TYPE_YAML_KEY]

        for params in force_list_params:
            yaml_dict[params] = force_list(yaml_dict.get(params, None))

        # Populate the model_metadata_s3_key values to handle both training and evaluation for all race_formats
        if None in yaml_dict[MODEL_METADATA_FILE_S3_YAML_KEY]:
            # MODEL_METADATA_FILE_S3_KEY not passed as part of yaml file ==> This happens during evaluation
            # Assume model_metadata.json is present in the s3_prefix/model/ folder
            yaml_dict[MODEL_METADATA_FILE_S3_YAML_KEY] = list()
            for s3_prefix in yaml_dict[MODEL_S3_PREFIX_YAML_KEY]:
                yaml_dict[MODEL_METADATA_FILE_S3_YAML_KEY].append(
                    os.path.join(s3_prefix, 'model/model_metadata.json'))

        # Validate the yaml values
        validate_yaml_values(yaml_dict)
        if os.path.exists(local_queue_pickle_path):
            with open(local_queue_pickle_path, 'rb') as f:
                tournament_candidate_queue = pickle.load(f)
            with open(local_report_pickle_path, 'rb') as f:
                tournament_report = pickle.load(f)
            logger.info('tournament_candidate_queue loaded from existing file')
        else:
            logger.info('tournament_candidate_queue initialized')
            tournament_candidate_queue = deque()
            for agent_idx, _ in enumerate(yaml_dict[MODEL_S3_BUCKET_YAML_KEY]):
                tournament_candidate_queue.append((
                    yaml_dict[MODEL_S3_BUCKET_YAML_KEY][agent_idx],
                    yaml_dict[MODEL_S3_PREFIX_YAML_KEY][agent_idx],
                    yaml_dict[MODEL_METADATA_FILE_S3_YAML_KEY][agent_idx],
                    yaml_dict[METRICS_S3_BUCKET_YAML_KEY][agent_idx],
                    yaml_dict[METRICS_S3_PREFIX_YAML_KEY][agent_idx],
                    yaml_dict[SIMTRACE_S3_BUCKET_YAML_KEY][agent_idx],
                    yaml_dict[SIMTRACE_S3_PREFIX_YAML_KEY][agent_idx],
                    yaml_dict[MP4_S3_BUCKET_YAML_KEY][agent_idx],
                    yaml_dict[MP4_S3_PREFIX_YAML_KEY][agent_idx],
                    yaml_dict[DISPLAY_NAME_YAML_KEY][agent_idx],
                    # TODO: Deprecate the DISPLAY_NAME and use only the RACER_NAME without if else check
                    "" if None in yaml_dict[RACER_NAME_YAML_KEY] else yaml_dict[RACER_NAME_YAML_KEY][agent_idx],
                    BodyShellType.F1_2021.value if yaml_dict[RACER_NAME_YAML_KEY][agent_idx] in F1_SHELL_USERS_LIST else BodyShellType.DEFAULT.value
                ))
            tournament_report = {"race_results": []}

        race_idx = len(tournament_report["race_results"])
        while len(tournament_candidate_queue) > 1:
            car1 = tournament_candidate_queue.popleft()
            car2 = tournament_candidate_queue.popleft()
            (car1_model_s3_bucket, car1_s3_prefix, car1_model_metadata,
             car1_metrics_bucket, car1_metrics_s3_key,
             car1_simtrace_bucket, car1_simtrace_prefix,
             car1_mp4_bucket, car1_mp4_prefix,
             car1_display_name, car1_racer_name,
             car1_body_shell_type) = car1
            (car2_model_s3_bucket, car2_s3_prefix, car2_model_metadata,
             car2_metrics_bucket, car2_metrics_s3_key,
             car2_simtrace_bucket, car2_simtrace_prefix,
             car2_mp4_bucket, car2_mp4_prefix,
             car2_display_name, car2_racer_name,
             car2_body_shell_type) = car2

            race_yaml_dict = generate_race_yaml(yaml_dict=yaml_dict, car1=car1, car2=car2,
                                                race_idx=race_idx)

            race_model_s3_buckets = [car1_model_s3_bucket, car2_model_s3_bucket]
            race_model_metadatas = [car1_model_metadata, car2_model_metadata]
            body_shell_types = [car1_body_shell_type, car2_body_shell_type]

            # List of directories created
            dirs_to_delete = list()
            yaml_dir = os.path.abspath(os.path.join(os.getcwd(), str(race_idx)))
            os.makedirs(yaml_dir)

            dirs_to_delete.append(yaml_dir)
            race_yaml_path = os.path.abspath(os.path.join(yaml_dir, 'evaluation_params.yaml'))
            with open(race_yaml_path, 'w') as race_yaml_file:
                yaml.dump(race_yaml_dict, race_yaml_file)

            # List of racecar names that should include second camera while launching
            racecars_with_stereo_cameras = list()
            # List of racecar names that should include lidar while launching
            racecars_with_lidars = list()
            # List of SimApp versions
            simapp_versions = list()
            for agent_index, model_s3_bucket in enumerate(race_model_s3_buckets):
                racecar_name = 'racecar_' + str(agent_index)
                # Make a local folder with the racecar name to download the model_metadata.json
                os.makedirs(os.path.join(os.getcwd(), racecar_name))
                dirs_to_delete.append(os.path.join(os.getcwd(), racecar_name))
                local_model_metadata_path = os.path.abspath(
                    os.path.join(os.path.join(os.getcwd(), racecar_name), 'model_metadata.json'))
                json_key = race_model_metadatas[agent_index]
                json_key = json_key.replace('s3://{}/'.format(model_s3_bucket), '')
                try:
                    s3_client.download_file(Bucket=model_s3_bucket,
                                            Key=json_key,
                                            Filename=local_model_metadata_path)
                except Exception as e:
                    log_and_exit("Failed to download model_metadata file: s3_bucket: {}, s3_key: {}, {}"
                                     .format(model_s3_bucket, json_key, e),
                                 SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                 SIMAPP_EVENT_ERROR_CODE_500)
                sensors, _, simapp_version = utils_parse_model_metadata.parse_model_metadata(local_model_metadata_path)
                simapp_versions.append(simapp_version)
                if Input.STEREO.value in sensors:
                    racecars_with_stereo_cameras.append(racecar_name)
                if Input.LIDAR.value in sensors or Input.SECTOR_LIDAR.value in sensors:
                    racecars_with_lidars.append(racecar_name)

            cmd = [os.path.join(os.path.dirname(os.path.abspath(__file__)), "tournament_race_node.py"),
                   str(race_idx),
                   race_yaml_path,
                   ','.join(racecars_with_stereo_cameras),
                   ','.join(racecars_with_lidars),
                   ','.join(simapp_versions),
                   ','.join(body_shell_types)
                   ]
            try:
                return_code, _, stderr = run_cmd(cmd_args=cmd,
                                                 shell=False,
                                                 stdout=None,
                                                 stderr=None)
            except KeyboardInterrupt:
                logger.info("KeyboardInterrupt raised, SimApp must be faulted! exiting...")
                return

            # Retrieve winner and append tournament report
            with open('race_report.pkl', 'rb') as f:
                race_report = pickle.load(f)
            race_report['race_idx'] = race_idx
            winner = car1 if race_report['winner'] == car1_display_name else car2
            logger.info("race {}'s winner: {}".format(race_idx, race_report['winner']))

            tournament_candidate_queue.append(winner)
            tournament_report["race_results"].append(race_report)

            # Clean up directories created
            for dir_to_delete in dirs_to_delete:
                shutil.rmtree(dir_to_delete, ignore_errors=True)
            race_idx += 1

            s3_extra_args = get_s3_kms_extra_args()
            # Persist latest queue and report to use after job restarts.
            with open(local_queue_pickle_path, 'wb') as f:
                pickle.dump(tournament_candidate_queue, f, protocol=2)
            s3_client.upload_file(Filename=local_queue_pickle_path,
                                  Bucket=s3_bucket,
                                  Key=queue_pickle_s3_key, ExtraArgs=s3_extra_args)

            with open(local_report_pickle_path, 'wb') as f:
                pickle.dump(tournament_report, f, protocol=2)
            s3_client.upload_file(Filename=local_report_pickle_path,
                                  Bucket=s3_bucket,
                                  Key=report_pickle_s3_key, ExtraArgs=s3_extra_args)

            # If there is more than 1 candidates then restart the simulation job otherwise
            # tournament is finished, persists final report and ends the job.
            if len(tournament_candidate_queue) > 1:
                restart_simulation_job(os.environ.get('AWS_ROBOMAKER_SIMULATION_JOB_ARN'),
                                       s3_region)
                break
            else:
                # Persist final tournament report in json format
                # and terminate the job by canceling it
                s3_client.put_object(Bucket=s3_bucket,
                                     Key=final_report_s3_key,
                                     Body=json.dumps(tournament_report), **s3_extra_args)

                cancel_simulation_job(os.environ.get('AWS_ROBOMAKER_SIMULATION_JOB_ARN'),
                                      s3_region)
    except Exception as e:
        log_and_exit("Tournament node failed: {}".format(e),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)


def is_power_of_two(n):
    """Return True if n is a power of two."""
    if n <= 0:
        return False
    else:
        return n & (n - 1) == 0


def validate_yaml_values(yaml_values):
    """ Validate that the parameter provided in the yaml file for configuration is correct.
    Some of the params requires list of two values. This is mostly checked as part of this function
    Arguments:
        yaml_values {[dict]} -- [All the yaml parameter as a list]
        multicar {[bool]} -- [Is multicar enabled (True), else False]
    Raises:
        Exception -- [Exception]
    """
    # Verify if all the yaml keys required for launching models have same number of values
    # TODO: Deprecate the DISPLAY_NAME and use only the RACER_NAME after cloud pushes this YAML parameter
    same_len_values = [MODEL_S3_BUCKET_YAML_KEY, MODEL_S3_PREFIX_YAML_KEY, MODEL_METADATA_FILE_S3_YAML_KEY,
                       METRICS_S3_BUCKET_YAML_KEY, METRICS_S3_PREFIX_YAML_KEY,
                       SIMTRACE_S3_BUCKET_YAML_KEY, SIMTRACE_S3_PREFIX_YAML_KEY,
                       MP4_S3_BUCKET_YAML_KEY, MP4_S3_PREFIX_YAML_KEY,
                       DISPLAY_NAME_YAML_KEY]
    if None not in yaml_values[RACER_NAME_YAML_KEY]:
        same_len_values.append(RACER_NAME_YAML_KEY)
    logger.info(yaml_values)
    if not all(map(lambda param: len(yaml_values[param]) == len(yaml_values[same_len_values[0]]), same_len_values)):
        logger.info("YAML parameter dimensions are not matching:")
        for val in same_len_values:
            logger.info("{}: {}".format(val,len(yaml_values[val])))
        raise Exception('Incorrect number of values for these yaml parameters {}'.format(same_len_values))

    # Verify if all yaml keys have power of 2 number of values.
    if not is_power_of_two(len(yaml_values[MODEL_S3_PREFIX_YAML_KEY])):
        raise Exception(
            'Incorrect number of values for tournament (Requires power of 2): Given {} '.format(len(yaml_values[MODEL_S3_PREFIX_YAML_KEY])))


if __name__ == '__main__':
    time.sleep(WAIT_FOR_ROBOMAKER_TIME)
    main()
