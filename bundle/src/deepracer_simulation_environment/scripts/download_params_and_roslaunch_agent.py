#!/usr/bin/env python3

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
import rospy

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
 
# Amount of time to wait to guarantee that RoboMaker's network configuration is ready.
WAIT_FOR_ROBOMAKER_TIME = 10
 
# User alias for F1 shell
F1_SHELL_USERS_LIST = ["TataCalde", "RIC3", "SheBangsTheDrums1989"]

def main():
    """ Main function for downloading yaml params """

    # parse argument
    s3_region = sys.argv[1]
    s3_bucket = sys.argv[2]
    s3_prefix = sys.argv[3]
    s3_yaml_name = sys.argv[4]
    launch_name = sys.argv[5]
    yaml_key = os.path.normpath(os.path.join(s3_prefix, s3_yaml_name))

    try:

        s3_endpoint_url = os.environ.get("S3_ENDPOINT_URL", None)
        
        if s3_endpoint_url is not None:
            logging.info('Endpoint URL {}'.format(s3_endpoint_url))
            rospy.set_param('S3_ENDPOINT_URL', s3_endpoint_url)

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

            cmd = [''.join(("roslaunch deepracer_simulation_environment {} ".format(launch_name),
                            "local_yaml_path:={} ".format(yaml_file.local_path),
                            "racecars_with_stereo_cameras:={} ".format(','.join(racecars_with_stereo_cameras)),
                            "racecars_with_lidars:={} ".format(','.join(racecars_with_lidars)),
                            "multicar:={} ".format(yaml_file.is_multicar),
                            "body_shell_types:={} ".format(','.join(yaml_file.body_shell_types)),
                            "simapp_versions:={} ".format(','.join(simapp_versions)),
                            "f1:={} ".format(yaml_file.is_f1),
                            "publish_to_kinesis_stream:={} ".format(str2bool(os.environ.get("ENABLE_KINESIS")))))]
        else:
            # Note: SimApp Version is default to 5.0: virtual event only have a single body_shell_types
            cmd = [''.join(("roslaunch deepracer_simulation_environment {} ".format(launch_name),
                            "local_yaml_path:={} ".format(yaml_file.local_path),
                            "simapp_versions:={} ".format('5.0'),                            
                            "multicar:={} ".format(yaml_file.is_multicar),
                            "kinesis_webrtc_signaling_channel_names:={} ".format(
                                ','.join(yaml_file.kinesis_webrtc_signaling_channel_name)),
                            "publish_to_kinesis_stream:={} ".format(str2bool(os.environ.get("ENABLE_KINESIS")))))]

        Popen(cmd, shell=True, executable="/bin/bash")
    
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
        pid_list = get_pid_list(["roslaunch", "rosmaster", "roscore"])  # ros processes
        pid_list.append(str(os.getpid()))
        # Get the ros processes pids to clean up
        with open(SAGEONLY_SIMAPP_JOB_PID_FILE_PATH, "w") as fp:
            for pid in pid_list:
                fp.write("%s\n" % pid)
            LOG.info("Writing sim job pid %s to %s", pid_list, SAGEONLY_SIMAPP_JOB_PID_FILE_PATH)


def get_pid_list(cmd_names):
    """return a list of pids that has the command given in the command names.

    Args:
        cmd_names ([str]): list of command names

    Returns:
        list: list of pids
    """
    pid_list = []
    sub_proc = Popen(['ps', 'aux'], shell=False, stdout=PIPE)
    # Discard the first line (ps aux header)
    sub_proc.stdout.readline()
    for line in sub_proc.stdout:
        # the separator for splitting is 'variable number of spaces'
        proc_info = re.split("\s+", line.decode('utf-8'), 10)
        pid = proc_info[1]
        cmd = proc_info[10]
        for cmd_name in cmd_names:
            if cmd_name in cmd:
                pid_list.append(pid)
    return pid_list


if __name__ == '__main__':
    write_pids_to_file()
    rospy.init_node('download_params_and_roslaunch_agent_node', anonymous=True)
    main()
    rospy.spin()
