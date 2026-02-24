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

"""Interface class to edit the images for different race_type
"""
import abc

class ImageEditingInterface(object):
    """ Interface to edit the images for different race_type
    Extends:
        metaclass=abc.ABCMeta
    """
    __metaclass__ = abc.ABCMeta
    @abc.abstractmethod
    def edit_image(self, major_cv_image, metric_info):
        """ Function to edit the images
        Decorators:
            abc.abstractmethod
        Arguments:
            major_cv_image (Image): The 45degree camera image following the car
            metric_info (Dict): Dict of agent_metric_info(List of ROS metric values of each agent) and training_phase
        Raises:
            NotImplementedError: Function to edit the image not implemented
        """
        raise NotImplementedError('Function to edit the image not implemented')

    def edit_top_camera_image(self, top_cv_image, metric_info):
        """ Function to edit the top camera image

        Arguments:
            top_cv_image (Image): The 45degree camera image following the car
            metric_info (Dict): Dict of agent_metric_info(List of ROS metric values of each agent) and training_phase
        Raises:
            NotImplementedError: Function to edit the top camera image not implemented
        """
        raise NotImplementedError('Function to edit the top camera image not implemented')
