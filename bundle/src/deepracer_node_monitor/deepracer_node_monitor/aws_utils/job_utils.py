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
""" Utils for the RoboMaker/Sagemaker functions"""
import os


class JobUtils(object):
    """
    Static class for robomaker/sagemaker utils
    """
    @staticmethod
    def get_simulation_arn() -> str:
        """
        Returns the RoboMaker/Sagemaker simulation arn set as environment variable when simulation is started

        Returns
            str: Environment variable set for AWS_ROBOMAKER_SIMULATION_JOB_ARN/AWS_SAGEMAKER_TRAINING_JOB_ARN
        """
        
        return os.environ.get('AWS_ROBOMAKER_SIMULATION_JOB_ARN', '')

    @staticmethod
    def get_job_id() -> str:
        """
        Parse the simulation id from the RoboMaker/Sagemaker ARN

        Returns
            str: Simulation ID for the RoboMaker/Sagemaker job
        """
        if(os.environ.get('SERVICE_TYPE','') == 'SAGEMAKER'):
            return os.environ.get('TRAINING_JOB_NAME', '')
        
        simulation_arn = JobUtils.get_simulation_arn()
        return simulation_arn.split("/")[-1]

    @staticmethod
    def get_aws_region() -> str:
        """
        Returns aws region from ACM_REGION env

        Returns
            str: returns aws_region from ACM_REGION env
        """
        return os.environ.get('ACM_REGION', 'us-east-1')
