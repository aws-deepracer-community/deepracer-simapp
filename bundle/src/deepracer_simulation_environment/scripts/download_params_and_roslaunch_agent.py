#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

"""script to download yaml param from S3 bucket to local directory and start training/eval ROS launch

Example:
    $ python download_params_and_roslaunch_agent.py s3_region, s3_bucket, s3_prefix, s3_yaml_name, launch_name
"""

import logging
from subprocess import Popen, PIPE
import os
import sys
import time
import botocore
import rclpy
from rclpy.node import Node

from enum import Enum
from markov.utils import test_internet_connection, str2bool
from markov.constants import DEFAULT_COLOR, DEEPRACER_JOB_TYPE_ENV, DeepRacerJobType
from markov.architecture.constants import Input
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                          SAGEONLY_SIMAPP_JOB_PID_FILE_PATH)
from markov.log_handler.exception_handler import log_and_exit
from markov.boto.s3.files.model_metadata import ModelMetadata
from markov.boto.s3.files.yaml_file import YamlFile
from markov.boto.s3.constants import (MODEL_METADATA_LOCAL_PATH_FORMAT, MODEL_METADATA_S3_POSTFIX,
                                      YAML_LOCAL_PATH_FORMAT,
                                      AgentType, YamlKey, ModelMetadataKeys)
from markov.boto.s3.utils import get_s3_key
from markov.log_handler.logger import Logger


LOG = Logger(__name__, logging.INFO).get_logger()


def main():
    """ Main function for downloading yaml params """
    # parse argument
    s3_region = sys.argv[1]
    s3_bucket = sys.argv[2]
    s3_prefix = sys.argv[3]
    s3_yaml_name = sys.argv[4]
    launch_name = sys.argv[5]

    try:

        s3_endpoint_url = os.environ.get("S3_ENDPOINT_URL", None)
        
        if AgentType.ROLLOUT.value in launch_name:
            # For rollout, launch_name is "rollout_rl_agent.launch"
            agent_type = AgentType.ROLLOUT.value
        elif AgentType.EVALUATION.value in launch_name:
            # For eval, launch_name is "evaluation_rl_agent.launch"
            agent_type = AgentType.EVALUATION.value
        elif AgentType.VIRTUAL_EVENT.value in launch_name:
            # For virtual event, launch_name is "virtual_event_rl_agent.launch"
            agent_type = AgentType.VIRTUAL_EVENT.value
        else:
            log_and_exit("Unknown agent type in launch file: {}".format(launch_name),
                         SIMAPP_SIMULATION_WORKER_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)

        # download yaml file
        yaml_key = get_s3_key(s3_prefix, s3_yaml_name)
        yaml_file = YamlFile(agent_type=agent_type,
                             bucket=s3_bucket,
                             s3_key=yaml_key,
                             region_name=s3_region,
                             s3_endpoint_url=s3_endpoint_url,
                             local_path=YAML_LOCAL_PATH_FORMAT.format(s3_yaml_name))
        yaml_file.get_yaml_values()

        if not agent_type == AgentType.VIRTUAL_EVENT.value:
            # List of racecar names that should include second camera while launching
            racecars_with_stereo_cameras = list()
            # List of racecar names that should include lidar while launching
            racecars_with_lidars = list()
            # List of SimApp versions
            simapp_versions = list()
            for agent_index, model_s3_bucket in enumerate(yaml_file.model_s3_buckets):
                racecar_name = 'racecar_{}'.format(agent_index) \
                    if yaml_file.is_multicar else 'racecar'
                json_key = yaml_file.model_metadata_s3_keys[agent_index]

                # download model metadata
                model_metadata = ModelMetadata(bucket=model_s3_bucket,
                                               s3_key=json_key,
                                               region_name=s3_region,
                                               s3_endpoint_url=s3_endpoint_url,
                                               local_path=MODEL_METADATA_LOCAL_PATH_FORMAT.format(racecar_name))
                model_metadata_info = model_metadata.get_model_metadata_info()
                sensors = model_metadata_info[ModelMetadataKeys.SENSOR.value]
                simapp_version = model_metadata_info[ModelMetadataKeys.VERSION.value]

                simapp_versions.append(str(simapp_version))
                if Input.STEREO.value in sensors:
                    racecars_with_stereo_cameras.append(racecar_name)
                if Input.LIDAR.value in sensors or Input.SECTOR_LIDAR.value in sensors or \
                        Input.DISCRETIZED_SECTOR_LIDAR.value in sensors:
                    racecars_with_lidars.append(racecar_name)

            cmd = [''.join(("ros2 launch deepracer_simulation_environment {} ".format(launch_name),
                            "local_yaml_path:={} ".format(yaml_file.local_path),
                            "racecars_with_stereo_cameras:={} ".format(','.join(racecars_with_stereo_cameras) or 'none'),
                            "racecars_with_lidars:={} ".format(','.join(racecars_with_lidars) or 'none'),
                            "multicar:={} ".format(yaml_file.is_multicar),
                            "body_shell_types:={} ".format(','.join(yaml_file.body_shell_types)),
                            "car_colors:={} ".format(','.join(yaml_file.car_colors)),
                            "simapp_versions:={} ".format(','.join(simapp_versions)),
                            "f1:={} ".format(yaml_file.is_f1),
                            "publish_to_kinesis_stream:={} ".format(str2bool(os.environ.get("ENABLE_KINESIS", "True"))),
                            "gui:={} ".format('true' if os.environ.get("ENABLE_GUI", None) == "True" else 'false')))]
        else:
            # Note: SimApp Version is default to 6.0: virtual event only have a single body_shell_types
            cmd = [''.join(("ros2 launch deepracer_simulation_environment {} ".format(launch_name),
                            "local_yaml_path:={} ".format(yaml_file.local_path),
                            "simapp_versions:={} ".format('6.0'),
                            "multicar:={} ".format(yaml_file.is_multicar),
                            "kinesis_webrtc_signaling_channel_names:={} ".format(
                                ','.join(yaml_file.kinesis_webrtc_signaling_channel_name)),
                            "publish_to_kinesis_stream:={} ".format(str2bool(os.environ.get("ENABLE_KINESIS", "True")))))]
        process = Popen(cmd, shell=True, executable="/bin/bash")
        LOG.info("Launched process pid %s for command: %s", process.pid, cmd[0])
        return process
    except botocore.exceptions.ClientError as ex:
        log_and_exit("Download params and launch of agent node S3 ClientError: s3_bucket: {}, yaml_key: {}, {}"
                         .format(s3_bucket, yaml_key, ex),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)
    except botocore.exceptions.EndpointConnectionError:
        log_and_exit("No Internet connection or s3 service unavailable",
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)
    except ValueError as ex:
        log_and_exit("User modified model_metadata.json: {}".format(ex),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)
    except Exception as ex:
        log_and_exit("Download params and launch of agent node failed: s3_bucket: {}, yaml_key: {}, {}"
                         .format(s3_bucket, yaml_key, ex),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)


def write_pids_to_file():
    """ Write pids to files for clean up purpose.
    """
    if os.environ.get(DEEPRACER_JOB_TYPE_ENV) == DeepRacerJobType.SAGEONLY.value:
        pid_list = get_pid_list(os.getpid())
        # Get the ros processes pids to clean up
        with open(SAGEONLY_SIMAPP_JOB_PID_FILE_PATH, "w") as fp:
            for pid in pid_list:
                fp.write("%s\n" % pid)
            LOG.info("Writing sim job pid %s to %s", pid_list, SAGEONLY_SIMAPP_JOB_PID_FILE_PATH)


def get_pid_list(root_pid):
    """Return a list containing root_pid and all descendant process pids.

    Args:
        root_pid (int): Root process pid.

    Returns:
        list: list of pids as strings
    """
    root_pid = str(root_pid)
    pid_to_children = {}

    sub_proc = Popen(['ps', '-eo', 'pid=', '-eo', 'ppid='], shell=False, stdout=PIPE)
    for line in sub_proc.stdout:
        parts = line.decode('utf-8').strip().split()
        if len(parts) != 2:
            continue
        pid, ppid = parts
        if ppid not in pid_to_children:
            pid_to_children[ppid] = []
        pid_to_children[ppid].append(pid)

    pid_list = []
    queue = [root_pid]
    seen = set()
    while queue:
        current_pid = queue.pop(0)
        if current_pid in seen:
            continue
        seen.add(current_pid)
        pid_list.append(current_pid)
        for child_pid in pid_to_children.get(current_pid, []):
            queue.append(child_pid)

    return pid_list


class DownloadParamsNode(Node):
    def __init__(self):
        super().__init__('download_params_and_roslaunch_agent_node')
        self.launch_process = None

    def set_launch_process(self, launch_process):
        self.launch_process = launch_process

    def monitor_launch_process(self):
        if self.launch_process is None:
            return

        if self.launch_process.poll() is not None:
            LOG.info("Launch process pid %s exited. Shutting down node.", self.launch_process.pid)
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def ros2_main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DownloadParamsNode()
        # Run the main download and launch logic
        launch_process = main()
        node.set_launch_process(launch_process)
        node.create_timer(10.0, node.monitor_launch_process)
        # Keep the node alive
        rclpy.spin(node)
    except Exception as ex:
        LOG.error(f"Exception in download params node: {ex}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    write_pids_to_file()
    ros2_main()
