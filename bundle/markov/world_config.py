# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

import os
import threading
from typing import Any
import logging
import yaml
from markov.log_handler.logger import Logger
from markov.boto.s3.files.yaml_file import YamlFile
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


logger = Logger(__name__, logging.INFO).get_logger()


OS_ENV_YAML_FILE = 'DR_MARKOV_WORLD_CONFIG_LOCAL_YAML_FILE'


class WorldConfig:
    """Singleton class to manage world configuration parameters """
    _instance = None
    _lock = threading.Lock()


    @staticmethod
    def get_instance():
        """ Get singleton instance of the World Config """
        with WorldConfig._lock:
            if WorldConfig._instance is None:
                WorldConfig._instance = WorldConfig()
            return WorldConfig._instance
    
    @staticmethod
    def get_param(parameter: str, default_value: Any) -> Any:
        """ get a parameter value """
        instance = WorldConfig.get_instance()

        if not instance._yaml_values:
            instance._load_yaml_file()
        
        if instance._yaml_values:
            if parameter in instance._yaml_values:
                return instance._yaml_values[parameter]
            else:
                logger.debug(f"Parameter {parameter} not found in yaml values")

        elif instance._throw_exception_missing_yaml_file():
            raise IOError("Unable to load yaml file for world config")

        return default_value

    @staticmethod 
    def set_param(parameter: str, value: Any) -> Any:
        """ set a parameter value """
        instance = WorldConfig.get_instance()

        if not instance._yaml_values:
            instance._load_yaml_file()
        
        if instance._yaml_values:
            instance._yaml_values[parameter] = value


    @staticmethod
    def get_env_var_local_yaml_file():
        return OS_ENV_YAML_FILE


    @staticmethod
    def get_launch_parameter(local_yaml_path_arg_name:str = 'local_yaml_path'):
        """ get a LaunchDescription parameter to set environment variable """
        return SetEnvironmentVariable(name=OS_ENV_YAML_FILE, value=LaunchConfiguration(local_yaml_path_arg_name))


    def __init__(self):
        self._yaml_values = None


    def set_local_yaml_config_file(self, local_yaml_file: str):
        """ set the local yaml config file """
        os.environ[OS_ENV_YAML_FILE] = local_yaml_file
        self._yaml_values = None


    def _load_yaml_file(self):
        """ load the world config parameters from the local yaml file """
        local_path = os.environ.get(OS_ENV_YAML_FILE, None)
        
        if not local_path:
            return False

        success = True
        try:
            with open(local_path, 'r') as yaml_file:
                self._yaml_values = yaml.safe_load(yaml_file)
        except IOError as e:
            success = False
        except yaml.YAMLError as e:
            success = False

        return success

    def _throw_exception_missing_yaml_file(self):
        throw_exc = os.environ.get(OS_ENV_YAML_FILE + "_MISSING_FAIL", 'false')
        return throw_exc in ('true', '1', 'True')


