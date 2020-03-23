#!/usr/bin/env python3

"""script to download yaml param from S3 bucket to local directory and start training/eval ROS launch

Example:
    $ python download_params_and_roslaunch_agent.py s3_region, s3_bucket, s3_prefix, s3_yaml_name, launch_name
"""

# import argparse
import logging
from subprocess import Popen
import os
import sys
import time
import botocore
import boto3
import yaml
import rospy
from markov import utils_parse_model_metadata
from markov.utils import force_list
from markov.architecture.constants import Input
from markov.utils import Logger, log_and_exit, get_boto_config
from markov.utils import (DEFAULT_COLOR,
                          SIMAPP_EVENT_ERROR_CODE_400,
                          SIMAPP_EVENT_ERROR_CODE_500,
                          SIMAPP_SIMULATION_WORKER_EXCEPTION)

LOG = Logger(__name__, logging.INFO).get_logger()

# Pass a list with 2 values for CAR_COLOR, MODEL_S3_BUCKET, MODEL_S3_PREFIX, MODEL_METADATA_FILE_S3_KEY for multicar
CAR_COLOR_YAML_KEY = "CAR_COLOR"
RACE_TYPE_YAML_KEY = "RACE_TYPE"
HEAD_TO_MODEL_RACE_TYPE = "HEAD_TO_MODEL"
TIME_TRIAL_RACE_TYPE = "TIME_TRIAL"
MODEL_S3_BUCKET_YAML_KEY = "MODEL_S3_BUCKET"
MODEL_S3_PREFIX_YAML_KEY = "MODEL_S3_PREFIX"
MODEL_METADATA_FILE_S3_YAML_KEY = "MODEL_METADATA_FILE_S3_KEY"
# Amount of time to wait to guarantee that RoboMaker's network configuration is ready.
WAIT_FOR_ROBOMAKER_TIME = 10

def main():
    """ Main function for downloading yaml params """
    try:
        # parse argument
        s3_region = sys.argv[1]
        s3_bucket = sys.argv[2]
        s3_prefix = sys.argv[3]
        s3_yaml_name = sys.argv[4]
        launch_name = sys.argv[5]

        # create boto3 session/client and download yaml/json file
        session = boto3.session.Session()

        s3_endpoint_url = os.environ.get("S3_ENDPOINT_URL", None)
        
        if s3_endpoint_url is not None:
            LOG.info('Endpoint URL {}'.format(s3_endpoint_url))
            rospy.set_param('S3_ENDPOINT_URL', s3_endpoint_url)
        else:
             # create boto3 session/client and download yaml/json file
            ec2_client = session.client('ec2', s3_region)
            LOG.info('Checking internet connection...')
            response = ec2_client.describe_vpcs()
            if not response['Vpcs']:
                log_and_exit("No VPC attached to instance", SIMAPP_SIMULATION_WORKER_EXCEPTION,
                             SIMAPP_EVENT_ERROR_CODE_500)
            LOG.info('Verified internet connection')

        s3_client = session.client('s3', region_name=s3_region, endpoint_url=s3_endpoint_url, config=get_boto_config())

        yaml_key = os.path.normpath(os.path.join(s3_prefix, s3_yaml_name))
        local_yaml_path = os.path.abspath(os.path.join(os.getcwd(), s3_yaml_name))
        s3_client.download_file(Bucket=s3_bucket, Key=yaml_key, Filename=local_yaml_path)
        # Get values passed in yaml files. Default values are for backward compatibility and for single racecar racing
        default_yaml_values = {RACE_TYPE_YAML_KEY: TIME_TRIAL_RACE_TYPE,
                               MODEL_S3_BUCKET_YAML_KEY: s3_bucket,
                               MODEL_S3_PREFIX_YAML_KEY: s3_prefix,
                               CAR_COLOR_YAML_KEY: DEFAULT_COLOR,
                               MODEL_METADATA_FILE_S3_YAML_KEY: None}
        yaml_dict = get_yaml_dict(local_yaml_path)
        yaml_values = get_yaml_values(yaml_dict, default_yaml_values)

        # Forcing the yaml parameter to list
        force_list_params = [MODEL_METADATA_FILE_S3_YAML_KEY, MODEL_S3_BUCKET_YAML_KEY, MODEL_S3_PREFIX_YAML_KEY,
                             CAR_COLOR_YAML_KEY]

        for params in force_list_params:
            yaml_values[params] = force_list(yaml_values[params])

        # Populate the model_metadata_s3_key values to handle both training and evaluation for all race_formats
        if None in yaml_values[MODEL_METADATA_FILE_S3_YAML_KEY]:
            # MODEL_METADATA_FILE_S3_KEY not passed as part of yaml file ==> This happens during evaluation
            # Assume model_metadata.json is present in the s3_prefix/model/ folder
            yaml_values[MODEL_METADATA_FILE_S3_YAML_KEY] = list()
            for s3_prefix in yaml_values[MODEL_S3_PREFIX_YAML_KEY]:
                yaml_values[MODEL_METADATA_FILE_S3_YAML_KEY].append(os.path.join(s3_prefix, 'model/model_metadata.json'))

        # Set multicar value if its a head to model racetype
        multicar = yaml_values[RACE_TYPE_YAML_KEY] == HEAD_TO_MODEL_RACE_TYPE
        # Validate the yaml values
        validate_yaml_values(yaml_values, multicar)
        # List of racecar names that should include second camera while launching
        racecars_with_stereo_cameras = list()

        # List of racecar names that should include lidar while launching
        racecars_with_lidars = list()

        # List of SimApp versions
        simapp_versions = list()

        for agent_index, model_s3_bucket in enumerate(yaml_values[MODEL_S3_BUCKET_YAML_KEY]):

            racecar_name = 'racecar_'+str(agent_index) if len(yaml_values[MODEL_S3_BUCKET_YAML_KEY]) > 1 else 'racecar'
            # Make a local folder with the racecar name to download the model_metadata.json
            if not os.path.exists(os.path.join(os.getcwd(), racecar_name)):
                os.makedirs(os.path.join(os.getcwd(), racecar_name))
            local_model_metadata_path = os.path.abspath(os.path.join(os.path.join(os.getcwd(), racecar_name),
                                                                     'model_metadata.json'))
            json_key = yaml_values[MODEL_METADATA_FILE_S3_YAML_KEY][agent_index]
            json_key = json_key.replace('s3://{}/'.format(model_s3_bucket), '')
            s3_client.download_file(Bucket=model_s3_bucket, Key=json_key, Filename=local_model_metadata_path)
            sensors, _, simapp_version = utils_parse_model_metadata.parse_model_metadata(local_model_metadata_path)
            simapp_versions.append(simapp_version)
            if Input.STEREO.value in sensors:
                racecars_with_stereo_cameras.append(racecar_name)
            if Input.LIDAR.value in sensors or Input.SECTOR_LIDAR.value in sensors:
                racecars_with_lidars.append(racecar_name)

        cmd = [''.join(("roslaunch deepracer_simulation_environment {} ".format(launch_name),
                        "local_yaml_path:={} ".format(local_yaml_path),
                         "racecars_with_stereo_cameras:={} ".format(','.join(racecars_with_stereo_cameras)),
                        "racecars_with_lidars:={} multicar:={} ".format(','.join(racecars_with_lidars), multicar),
                        "car_colors:={} simapp_versions:={}".format(','.join(yaml_values[CAR_COLOR_YAML_KEY]),
                                                                    ','.join(simapp_versions))))]
        Popen(cmd, shell=True, executable="/bin/bash")
    
    except botocore.exceptions.ClientError as ex:
        log_and_exit("Download params and launch of agent node failed: s3_bucket: {}, yaml_key: {}, {}".format(s3_bucket, yaml_key, ex),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION, SIMAPP_EVENT_ERROR_CODE_400)
    except botocore.exceptions.EndpointConnectionError:
        log_and_exit("No Internet connection or s3 service unavailable", SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)
    except Exception as ex:
        log_and_exit("Download params and launch of agent node failed: s3_bucket: {}, yaml_key: {}, {}".format(s3_bucket, yaml_key, ex),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION, SIMAPP_EVENT_ERROR_CODE_500)

def validate_yaml_values(yaml_values, multicar):
    """ Validate that the parameter provided in the yaml file for configuration is correct.
    Some of the params requires list of two values. This is mostly checked as part of this function
    Arguments:
        yaml_values {[dict]} -- [All the yaml parameter as a list]
        multicar {[bool]} -- [Is multicar enabled (True), else False]
    Raises:
        Exception -- [Exception]
    """
    # Verify if all the yaml keys required for launching models have same number of values
    same_len_values = [MODEL_S3_BUCKET_YAML_KEY, MODEL_S3_PREFIX_YAML_KEY, MODEL_METADATA_FILE_S3_YAML_KEY,
                       CAR_COLOR_YAML_KEY]
    LOG.info(yaml_values)
    if not all(map(lambda param: len(yaml_values[param]) == len(yaml_values[same_len_values[0]]), same_len_values)):
        raise Exception('Incorrect number of values for these yaml parameters {}'.format(same_len_values))

    # Verify if all yaml keys have 2 values for multi car racing
    if multicar and len(yaml_values[MODEL_S3_PREFIX_YAML_KEY]) != 2:
        raise Exception('Incorrect number of values for multicar racing yaml parameters {}'.format(same_len_values))

    # Verify if all yaml keys have 1 value for single car racing
    if not multicar and len(yaml_values[MODEL_S3_PREFIX_YAML_KEY]) != 1:
        raise Exception('Incorrect number of values for single car racing yaml parameters {}'.format(same_len_values))


def get_yaml_dict(local_yaml_path):
    '''local_yaml_path - path to the local yaml file
    '''
    with open(local_yaml_path, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            log_and_exit("yaml read error: {}".format(exc),
                         SIMAPP_SIMULATION_WORKER_EXCEPTION, SIMAPP_EVENT_ERROR_CODE_500)


def get_yaml_values(yaml_dict, default_vals=None):
    '''yaml_dict - dict containing yaml configs
       default_vals - Dictionary of the default values to be used if key is not present
    '''
    try:
        return_values = dict()
        default_val_keys = default_vals.keys() if default_vals else []
        for key in default_val_keys:
            return_values[key] = yaml_dict.get(key, default_vals[key])
        return return_values
    except yaml.YAMLError as exc:
        log_and_exit("yaml read error: {}".format(exc),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION, SIMAPP_EVENT_ERROR_CODE_500)



if __name__ == '__main__':
    time.sleep(WAIT_FOR_ROBOMAKER_TIME)
    rospy.init_node('download_params_and_roslaunch_agent_node', anonymous=True)
    main()
    rospy.spin()
