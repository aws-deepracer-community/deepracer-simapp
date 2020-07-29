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

from markov.rollout_constants import BodyShellType
from markov.utils import (force_list, restart_simulation_job,
                          cancel_simulation_job, get_boto_config, get_s3_kms_extra_args)
from markov.architecture.constants import Input
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_400,
                                          SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_WORKER_EXCEPTION)
from markov.s3.files.model_metadata import ModelMetadata
from markov.s3.files.yaml_file import YamlFile
from markov.s3.constants import (MODEL_METADATA_LOCAL_PATH_FORMAT, MODEL_METADATA_S3_POSTFIX,
                                 YAML_LOCAL_PATH_FORMAT, AgentType, YamlKey)
from markov.s3.utils import get_s3_key

logger = Logger(__name__, logging.INFO).get_logger()
# Amount of time to wait to guarantee that RoboMaker's network configuration is ready.
WAIT_FOR_ROBOMAKER_TIME = 10

RACE_CAR_COLORS = ["Purple", "Orange"]


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

    race_yaml_dict[YamlKey.CAR_COLOR_YAML_KEY.value] = RACE_CAR_COLORS
    race_yaml_dict[YamlKey.MODEL_S3_BUCKET_YAML_KEY.value] = \
        [car1_model_s3_bucket, car2_model_s3_bucket]
    race_yaml_dict[YamlKey.MODEL_S3_PREFIX_YAML_KEY.value] = \
        [car1_s3_prefix, car2_s3_prefix]
    race_yaml_dict[YamlKey.MODEL_METADATA_FILE_S3_YAML_KEY.value] = \
        [car1_model_metadata, car2_model_metadata]
    race_yaml_dict[YamlKey.METRICS_S3_BUCKET_YAML_KEY.value] = \
        [car1_metrics_bucket, car2_metrics_bucket]
    race_yaml_dict[YamlKey.METRICS_S3_OBJECT_KEY_YAML_KEY.value] = \
        [os.path.join(car1_metrics_s3_prefix, str(uuid.uuid4()), 'evaluation_metrics.json'),
         os.path.join(car2_metrics_s3_prefix, str(uuid.uuid4()), 'evaluation_metrics.json')]
    race_yaml_dict[YamlKey.SIMTRACE_S3_BUCKET_YAML_KEY.value] = \
        [car1_simtrace_bucket, car2_simtrace_bucket]
    race_yaml_dict[YamlKey.SIMTRACE_S3_PREFIX_YAML_KEY.value] = \
        [os.path.join(car1_simtrace_prefix, str(uuid.uuid4())),
         os.path.join(car2_simtrace_prefix, str(uuid.uuid4()))]
    race_yaml_dict[YamlKey.MP4_S3_BUCKET_YAML_KEY.value] = \
        [car1_mp4_bucket, car2_mp4_bucket]
    race_yaml_dict[YamlKey.MP4_S3_PREFIX_YAML_KEY.value] = \
        [os.path.join(car1_mp4_prefix, str(uuid.uuid4())),
         os.path.join(car2_mp4_prefix, str(uuid.uuid4()))]
    race_yaml_dict[YamlKey.DISPLAY_NAME_YAML_KEY.value] = \
        [car1_display_name, car2_display_name]
    # TODO - Deprecate the DISPLAY_NAME once cloud team pushes the RACER_NAME yaml
    race_yaml_dict[YamlKey.RACER_NAME_YAML_KEY.value] = [car1_racer_name, car2_racer_name]
    race_yaml_dict[YamlKey.BODY_SHELL_TYPE_YAML_KEY.value] = \
        [car1_body_shell_type, car2_body_shell_type]
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

        # download yaml file
        yaml_file = YamlFile(agent_type=AgentType.TOURNAMENT.value,
                             bucket=s3_bucket,
                             s3_key=get_s3_key(s3_prefix, s3_yaml_name),
                             region_name=s3_region,
                             s3_endpoint_url=s3_endpoint_url,
                             local_path=YAML_LOCAL_PATH_FORMAT.format(s3_yaml_name))

        yaml_dict = yaml_file.get_yaml_values()

        if os.path.exists(local_queue_pickle_path):
            with open(local_queue_pickle_path, 'rb') as f:
                tournament_candidate_queue = pickle.load(f)
            with open(local_report_pickle_path, 'rb') as f:
                tournament_report = pickle.load(f)
            logger.info('tournament_candidate_queue loaded from existing file')
        else:
            logger.info('tournament_candidate_queue initialized')
            tournament_candidate_queue = deque()
            for agent_idx, _ in enumerate(yaml_dict[YamlKey.MODEL_S3_BUCKET_YAML_KEY.value]):
                tournament_candidate_queue.append((
                    yaml_dict[YamlKey.MODEL_S3_BUCKET_YAML_KEY.value][agent_idx],
                    yaml_dict[YamlKey.MODEL_S3_PREFIX_YAML_KEY.value][agent_idx],
                    yaml_dict[YamlKey.MODEL_METADATA_FILE_S3_YAML_KEY.value][agent_idx],
                    yaml_dict[YamlKey.METRICS_S3_BUCKET_YAML_KEY.value][agent_idx],
                    yaml_dict[YamlKey.METRICS_S3_PREFIX_YAML_KEY.value][agent_idx],
                    yaml_dict[YamlKey.SIMTRACE_S3_BUCKET_YAML_KEY.value][agent_idx],
                    yaml_dict[YamlKey.SIMTRACE_S3_PREFIX_YAML_KEY.value][agent_idx],
                    yaml_dict[YamlKey.MP4_S3_BUCKET_YAML_KEY.value][agent_idx],
                    yaml_dict[YamlKey.MP4_S3_PREFIX_YAML_KEY.value][agent_idx],
                    yaml_dict[YamlKey.DISPLAY_NAME_YAML_KEY.value][agent_idx],
                    # TODO: Deprecate the DISPLAY_NAME and use only the RACER_NAME without if else check
                    "" if None in yaml_dict.get(YamlKey.RACER_NAME_YAML_KEY.value, [None]) \
                        else yaml_dict[YamlKey.RACER_NAME_YAML_KEY.value][agent_idx],
                    yaml_dict[YamlKey.BODY_SHELL_TYPE_YAML_KEY.value][agent_idx]
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
                json_key = race_model_metadatas[agent_index]
                # download model metadata
                try:
                    model_metadata = ModelMetadata(bucket=model_s3_bucket,
                                                   s3_key=json_key,
                                                   region_name=s3_region,
                                                   s3_endpoint_url=s3_endpoint_url,
                                                   local_path=MODEL_METADATA_LOCAL_PATH_FORMAT.format(racecar_name))
                    dirs_to_delete.append(model_metadata.local_dir)
                except Exception as e:
                    log_and_exit("Failed to download model_metadata file: s3_bucket: {}, s3_key: {}, {}"
                                 .format(model_s3_bucket, json_key, e),
                                 SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                 SIMAPP_EVENT_ERROR_CODE_500)
                sensors, _, simapp_version = model_metadata.get_model_metadata_info()
                simapp_versions.append(str(simapp_version))
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
    except ValueError as ex:
        log_and_exit("User modified model_metadata.json: {}".format(ex),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_400)
    except Exception as e:
        log_and_exit("Tournament node failed: {}".format(e),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)


if __name__ == '__main__':
    time.sleep(WAIT_FOR_ROBOMAKER_TIME)
    main()
