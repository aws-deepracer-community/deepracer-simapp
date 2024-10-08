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

"""this model handles sdf, urdf, and xacro xml file parsing"""

import os
import xacro

from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                          SIMAPP_EVENT_ERROR_CODE_500)


class GazeboXmlLoader(object):
    """GazeboXmlLoader to parse sdf, urdf, and xacro file"""
    @staticmethod
    def parse(file_path: str, **kwargs) -> str:
        """parse sdf, urdf, or xacro file

        Args:
            file_path (str): file path to parse
            **kwargs: Arbitrary keyword arguments

        Returns:
            str: string of processed file contents

        Raises:
            Exception: GazeboXmlLoader parse file loading or non-recognized type
            exception
        """
        _, file_extension = os.path.splitext(file_path)
        try:
            if file_extension in ['.sdf', '.urdf']:
                with open(file_path, "r") as file_pointer:
                    xml = file_pointer.read()
                return xml
            if file_extension == '.xacro':
                xacro_xml = xacro.process_file(input_file_name=file_path,
                                               mappings=kwargs).toxml()
                return xacro_xml
            log_and_exit("[GazeboXmlLoader]: file type {} not recognizable".format(file_extension),
                         SIMAPP_SIMULATION_WORKER_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)
        except Exception as ex:
            log_and_exit("[GazeboXmlLoader]: file open or parse failure, {}".format(ex),
                         SIMAPP_SIMULATION_WORKER_EXCEPTION,
                         SIMAPP_EVENT_ERROR_CODE_500)
