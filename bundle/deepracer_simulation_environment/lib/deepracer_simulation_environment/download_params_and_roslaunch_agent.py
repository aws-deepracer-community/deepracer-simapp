#!/usr/bin/env python3

"""script to download yaml param from S3 bucket to local directory and start training/eval ROS launch

Example:
    $ python download_params_and_roslaunch_agent.py s3_region, s3_bucket, s3_prefix, s3_yaml_name, launch_name
"""

import logging
from subprocess import Popen
import os
import sys
import time
import botocore
import rospy

from enum import Enum
from markov.utils import test_internet_connection
from markov.constants import DEFAULT_COLOR
from markov.architecture.constants import Input
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_400, SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_WORKER_EXCEPTION)
from markov.log_handler.exception_handler import log_and_exit
from markov.s3.files.model_metadata import ModelMetadata
from markov.s3.files.yaml_file import YamlFile
from markov.s3.constants import (MODEL_METADATA_LOCAL_PATH_FORMAT, MODEL_METADATA_S3_POSTFIX,
                                 YAML_LOCAL_PATH_FORMAT,
                                 AgentType, YamlKey)
from markov.s3.utils import get_s3_key
 
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
        # List of racecar names that should include second camera while launching
        racecars_with_stereo_cameras = list()
        # List of racecar names that should include lidar while launching
        racecars_with_lidars = list()
        # List of SimApp versions
        simapp_versions = list()

        for agent_index, model_s3_bucket in enumerate(yaml_file.model_s3_buckets):
            racecar_name = 'racecar_' + str(agent_index) \
                if yaml_file.is_multicar else 'racecar'
            json_key = yaml_file.model_metadata_s3_keys[agent_index]

            # download model metadata
            model_metadata = ModelMetadata(bucket=model_s3_bucket,
                                           s3_key=json_key,
                                           region_name=s3_region,
                                           s3_endpoint_url=s3_endpoint_url,
                                           local_path=MODEL_METADATA_LOCAL_PATH_FORMAT.format(racecar_name))
            sensors, _, simapp_version = model_metadata.get_model_metadata_info()            
            simapp_versions.append(str(simapp_version))
            if Input.STEREO.value in sensors:
                racecars_with_stereo_cameras.append(racecar_name)
            if Input.LIDAR.value in sensors or Input.SECTOR_LIDAR.value in sensors:
                racecars_with_lidars.append(racecar_name)

        cmd = [''.join(("roslaunch deepracer_simulation_environment {} ".format(launch_name),
                        "local_yaml_path:={} ".format(yaml_file.local_path),
                        "racecars_with_stereo_cameras:={} ".format(','.join(racecars_with_stereo_cameras)),
                        "racecars_with_lidars:={} ".format(','.join(racecars_with_lidars)),
                        "multicar:={} ".format(yaml_file.is_multicar),
                        "body_shell_types:={} ".format(','.join(yaml_file.body_shell_types)),
                        "simapp_versions:={} ".format(','.join(simapp_versions)),
                        "f1:={}".format(yaml_file.is_f1)))]
        Popen(cmd, shell=True, executable="/bin/bash")
    
    except botocore.exceptions.ClientError as ex:
        log_and_exit("Download params and launch of agent node failed: s3_bucket: {}, yaml_key: {}, {}"
                         .format(s3_bucket, yaml_key, ex), 
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_400)
    except botocore.exceptions.EndpointConnectionError:
        log_and_exit("No Internet connection or s3 service unavailable",
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)
    except ValueError as ex:
        log_and_exit("User modified model_metadata.json: {}".format(ex),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_400)
    except Exception as ex:
        log_and_exit("Download params and launch of agent node failed: s3_bucket: {}, yaml_key: {}, {}"
                         .format(s3_bucket, yaml_key, ex), 
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)

if __name__ == '__main__':
    rospy.init_node('download_params_and_roslaunch_agent_node', anonymous=True)
    time.sleep(WAIT_FOR_ROBOMAKER_TIME)
    main()
    rospy.spin()
