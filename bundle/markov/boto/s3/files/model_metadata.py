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

'''This module implements s3 client for model metadata'''

import os
import logging
import json

from markov.log_handler.logger import Logger
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_WORKER_EXCEPTION)
from markov.log_handler.exception_handler import log_and_exit
from markov.architecture.constants import Input, NeuralNetwork
from markov.constants import SIMAPP_VERSION_2, SIMAPP_VERSION_1
from markov.boto.s3.constants import (ModelMetadataKeys,
                                      TrainingAlgorithm,
                                      ActionSpaceTypes,
                                      DiscretizedSectorLidarDefaults)
from markov.sensors.constants import (
    LIDAR_360_DEGREE_SAMPLE,
    LIDAR_360_DEGREE_MIN_RANGE,
    LIDAR_360_DEGREE_MAX_RANGE
)
from markov.boto.s3.s3_client import S3Client

LOG = Logger(__name__, logging.INFO).get_logger()


class ModelMetadata():
    '''model metadata file upload, download, and parse
    '''
    def __init__(self, bucket, s3_key, region_name="us-east-1",
                 s3_endpoint_url=None,
                 local_path="./custom_files/agent/model_metadata.json",
                 max_retry_attempts=5, backoff_time_sec=1.0):
        '''Model metadata upload, download, and parse

        Args:
            bucket (str): S3 bucket string
            s3_key: (str): S3 key string.
            region_name (str): S3 region name
            local_path (str): file local path
            max_retry_attempts (int): maximum number of retry attempts for S3 download/upload
            backoff_time_sec (float): backoff second between each retry

        '''
        # check s3 key and s3 bucket exist
        if not bucket or not s3_key:
            log_and_exit("model_metadata S3 key or bucket not available for S3. \
                         bucket: {}, key {}"
                         .format(bucket, s3_key),
                         SIMAPP_SIMULATION_WORKER_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)
        self._bucket = bucket
        # Strip the s3://<bucket> from uri, if s3_key past in as uri
        self._s3_key = s3_key.replace('s3://{}/'.format(self._bucket), '')
        self._local_path = local_path
        self._local_dir = os.path.dirname(self._local_path)
        self._model_metadata = None
        self._s3_client = S3Client(region_name,
                                   s3_endpoint_url,
                                   max_retry_attempts,
                                   backoff_time_sec)

    @property
    def local_dir(self):
        '''return local dir of model metadata'''
        return self._local_dir

    @property
    def local_path(self):
        '''return local path of model metadata'''
        return self._local_path

    @property
    def action_space(self):
        '''return action space values passed as part of model metadata'''
        # download model_metadata.json and after successfully download, parse and set the class variable
        if not self._model_metadata:
            self._download_and_set_model_metadata()
        return self._model_metadata[ModelMetadataKeys.ACTION_SPACE.value]

    @property
    def action_space_type(self):
        '''return action space type value passed as part of model metadata'''
        # download model_metadata.json and after successfully download, parse and set the class variable
        if not self._model_metadata:
            self._download_and_set_model_metadata()
        return self._model_metadata[ModelMetadataKeys.ACTION_SPACE_TYPE.value]

    @property
    def training_algorithm(self):
        '''return training algorithm value passed as part of model metadata'''
        # download model_metadata.json and after successfully download, parse and set the class variable
        if not self._model_metadata:
            self._download_and_set_model_metadata()
        return self._model_metadata[ModelMetadataKeys.TRAINING_ALGORITHM.value]

    @property
    def lidar_num_sectors(self):
        if not self._model_metadata:
            self._download_and_set_model_metadata()
        lidar_config_key = ModelMetadataKeys.LIDAR_CONFIG.value
        num_sectors_key = ModelMetadataKeys.NUM_SECTORS.value
        return self._model_metadata[lidar_config_key][num_sectors_key]

    @property
    def lidar_num_values_per_sector(self):
        if not self._model_metadata:
            self._download_and_set_model_metadata()
        lidar_config_key = ModelMetadataKeys.LIDAR_CONFIG.value
        num_values_per_sector_key = ModelMetadataKeys.NUM_VALUES_PER_SECTOR.value
        return self._model_metadata[lidar_config_key][num_values_per_sector_key]

    @property
    def lidar_clipping_dist(self):
        if not self._model_metadata:
            self._download_and_set_model_metadata()
        lidar_config_key = ModelMetadataKeys.LIDAR_CONFIG.value
        clipping_dist_key = ModelMetadataKeys.CLIPPING_DISTANCE.value
        return self._model_metadata[lidar_config_key][clipping_dist_key]

    def get_action_dict(self, action):
        """return the action dict containing the steering_angle and speed value

        Args:
            action (int or list): model metadata action_space index for discreet action spaces
                                  or [steering, speed] float values for continuous action spaces

        Returns:
            dict (str, float): dictionary containing {steering_angle: value, speed: value}
        """
        if self.action_space_type == ActionSpaceTypes.DISCRETE.value:
            return self._model_metadata[ModelMetadataKeys.ACTION_SPACE.value][action]
        elif self.action_space_type == ActionSpaceTypes.CONTINUOUS.value:
            json_action = dict()
            json_action[ModelMetadataKeys.STEERING_ANGLE.value] = action[0]
            json_action[ModelMetadataKeys.SPEED.value] = action[1]
            return json_action
        else:
            log_and_exit("Unknown action_space_type found while getting action dict. \
                action_space_type: {}".format(self.action_space_type),
                         SIMAPP_SIMULATION_WORKER_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)

    def _download_and_set_model_metadata(self):
        '''download and parse the model metadata file'''
        # download model_metadata.json
        self._download()
        # after successfully download or use default model metadata, then parse
        self._model_metadata = self.parse_model_metadata(self._local_path)

    def get_model_metadata_info(self):
        '''retrive the model metadata info

        Returns:
            dict: dictionary containing the information that is parsed from model_metatadata.json

        '''
        # download model_metadata.json and after successfully download, parse and set the class variable
        if not self._model_metadata:
            self._download_and_set_model_metadata()

        return self._model_metadata

    def persist(self, s3_kms_extra_args):
        '''upload local model_metadata.json into S3 bucket

        Args:
            s3_kms_extra_args (dict): s3 key management service extra argument

        '''

        # persist model metadata
        # if retry failed, s3_client upload_file will log and exit 500
        self._s3_client.upload_file(bucket=self._bucket,
                                    s3_key=self._s3_key,
                                    local_path=self._local_path,
                                    s3_kms_extra_args=s3_kms_extra_args)

    def _download(self):
        '''download model_metadata.json with retry from s3 bucket'''

        # check and make local directory
        if self._local_dir and not os.path.exists(self._local_dir):
            os.makedirs(self._local_dir)
        # download model metadata
        # if retry failed, each worker.py and download_params_and_roslaunch_agent.py
        # will handle 400 and 500 separately
        self._s3_client.download_file(bucket=self._bucket,
                                      s3_key=self._s3_key,
                                      local_path=self._local_path)
        LOG.info("[s3] Successfully downloaded model metadata \
                 from s3 key {} to local {}.".format(self._s3_key, self._local_path))

    @staticmethod
    def parse_model_metadata(local_model_metadata_path):
        """parse model metadata give the local path

        Args:
            local_model_metadata_path (str): local model metadata string

        Returns:
            dict (str, obj): dictionary of all required information parsed out of model_metadata.json file

        """
        model_metadata = dict()
        try:
            with open(local_model_metadata_path, "r") as json_file:
                data = json.load(json_file)
                # simapp_version 2.0+ should contain version as key in
                # model_metadata.json
                if ModelMetadataKeys.ACTION_SPACE.value not in data:
                    raise ValueError("no action space defined")
                action_values = data[ModelMetadataKeys.ACTION_SPACE.value]
                if ModelMetadataKeys.VERSION.value in data:
                    simapp_version = float(data[ModelMetadataKeys.VERSION.value])
                    if simapp_version >= SIMAPP_VERSION_2:
                        sensor = data[ModelMetadataKeys.SENSOR.value]
                    else:
                        sensor = [Input.OBSERVATION.value]
                else:
                    if ModelMetadataKeys.SENSOR.value in data:
                        sensor = data[ModelMetadataKeys.SENSOR.value]
                        simapp_version = SIMAPP_VERSION_2
                    else:
                        sensor = [Input.OBSERVATION.value]
                        simapp_version = SIMAPP_VERSION_1

                # Model Metadata Sensor validation
                camera_sensors = [Input.OBSERVATION.value, Input.CAMERA.value, Input.LEFT_CAMERA.value,
                                  Input.STEREO.value]
                camera_sensor_type_count = sum([sensor_elem in camera_sensors for sensor_elem in sensor])
                if camera_sensor_type_count != 1:
                    raise ValueError("model_metadata MUST contain only 1 camera sensor: {}".format(sensor))

                lidar_sensors = [Input.LIDAR.value, Input.SECTOR_LIDAR.value, Input.DISCRETIZED_SECTOR_LIDAR.value]
                lidar_sensor_type_count = sum([sensor_elem in lidar_sensors for sensor_elem in sensor])
                if lidar_sensor_type_count > 1:
                    raise ValueError("model_metadata contains more than 1 lidar sensor: {}".format(sensor))

                num_lidar_sectors_key = ModelMetadataKeys.NUM_SECTORS.value
                num_values_per_sector_key = ModelMetadataKeys.NUM_VALUES_PER_SECTOR.value
                clipping_dist_key = ModelMetadataKeys.CLIPPING_DISTANCE.value

                lidar_config = {num_lidar_sectors_key: DiscretizedSectorLidarDefaults.NUMBER_OF_SECTORS,
                                num_values_per_sector_key: DiscretizedSectorLidarDefaults.NUM_VALUES_PER_SECTOR,
                                clipping_dist_key: DiscretizedSectorLidarDefaults.CLIPPING_DIST}
                if ModelMetadataKeys.LIDAR_CONFIG.value in data:
                    config = data[ModelMetadataKeys.LIDAR_CONFIG.value]
                    if num_lidar_sectors_key in config:
                        config[num_lidar_sectors_key] = int(config[num_lidar_sectors_key])
                        if config[num_lidar_sectors_key] <= 0 or \
                                LIDAR_360_DEGREE_SAMPLE % config[num_lidar_sectors_key] != 0:
                            err_msg_format = "{} % Number of sector ({}) must be 0!"
                            err_msg = err_msg_format.format(LIDAR_360_DEGREE_SAMPLE,
                                                            config[num_lidar_sectors_key])
                            raise ValueError(err_msg)
                        lidar_config[num_lidar_sectors_key] = config[num_lidar_sectors_key]
                    if num_values_per_sector_key in config:
                        config[num_values_per_sector_key] = int(config[num_values_per_sector_key])
                        if config[num_values_per_sector_key] < 1:
                            err_msg_format = "Number of values per sector ({}) cannot be smaller than 1."
                            err_msg = err_msg_format.format(config[num_values_per_sector_key])
                            raise ValueError(err_msg)
                        lidar_config[num_values_per_sector_key] = config[num_values_per_sector_key]
                    if clipping_dist_key in config:
                        if LIDAR_360_DEGREE_MIN_RANGE > config[clipping_dist_key] > LIDAR_360_DEGREE_MAX_RANGE:
                            err_msg_format = "Clipping distance ({}) must be between {} and {} inclusively."
                            err_msg = err_msg_format.format(config[num_values_per_sector_key],
                                                            LIDAR_360_DEGREE_MIN_RANGE,
                                                            LIDAR_360_DEGREE_MAX_RANGE)
                            raise ValueError(err_msg)
                        lidar_config[clipping_dist_key] = config[clipping_dist_key]

                if ModelMetadataKeys.NEURAL_NETWORK.value in data:
                    network = data[ModelMetadataKeys.NEURAL_NETWORK.value]
                else:
                    network = NeuralNetwork.DEEP_CONVOLUTIONAL_NETWORK_SHALLOW.value
                training_algorithm = TrainingAlgorithm.CLIPPED_PPO.value
                if ModelMetadataKeys.TRAINING_ALGORITHM.value in data:
                    data_training_algorithm = data[ModelMetadataKeys.TRAINING_ALGORITHM.value].lower().strip()
                    # Update the training algorithm value if its valid else log and exit
                    if TrainingAlgorithm.has_training_algorithm(data_training_algorithm):
                        training_algorithm = data_training_algorithm
                    else:
                        log_and_exit("Unknown training_algorithm found while parsing model_metadata. \
                            training_algorithm: {}".format(data_training_algorithm),
                                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                     SIMAPP_EVENT_ERROR_CODE_500)
                action_space_type = ActionSpaceTypes.DISCRETE.value
                if ModelMetadataKeys.ACTION_SPACE_TYPE.value in data:
                    data_action_space_type = data[ModelMetadataKeys.ACTION_SPACE_TYPE.value].lower().strip()
                    # Update the training algorithm value if its valid else log and exit
                    if ActionSpaceTypes.has_action_space(data_action_space_type):
                        action_space_type = data_action_space_type
                    else:
                        log_and_exit("Unknown action_space_type found while parsing model_metadata. \
                            action_space_type: {}".format(data_action_space_type),
                                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                     SIMAPP_EVENT_ERROR_CODE_500)

            LOG.info("Sensor list %s, network %s, simapp_version %s, training_algorithm %s, action_space_type %s lidar_config %s",
                     sensor, network,
                     simapp_version, training_algorithm, action_space_type, lidar_config)
            model_metadata[ModelMetadataKeys.SENSOR.value] = sensor
            model_metadata[ModelMetadataKeys.NEURAL_NETWORK.value] = network
            model_metadata[ModelMetadataKeys.VERSION.value] = simapp_version
            model_metadata[ModelMetadataKeys.TRAINING_ALGORITHM.value] = training_algorithm
            model_metadata[ModelMetadataKeys.ACTION_SPACE.value] = action_values
            model_metadata[ModelMetadataKeys.ACTION_SPACE_TYPE.value] = action_space_type
            model_metadata[ModelMetadataKeys.LIDAR_CONFIG.value] = lidar_config
            return model_metadata
        except ValueError as ex:
            raise ValueError('model_metadata ValueError: {}'.format(ex))
        except Exception as ex:
            raise Exception('Model metadata does not exist: {}'.format(ex))
