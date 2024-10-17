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

'''This module defines the interface for metrics, this is the data we typically upload to
    s3
'''
import abc

class MetricsInterface(object, metaclass=abc.ABCMeta):
    def upload_episode_metrics(self):
        '''Uploads the desired episode metrics to s3
           metrics - Dictionary of metrics to upload
        '''
        raise NotImplementedError('Metrics class must be able to upload episode metrics')

    def upload_step_metrics(self, metrics):
        '''Uploads step metrics to s3
           metrics - Dictionary of metrics to upload
        '''
        raise NotImplementedError('Metrics class must be able to upload step metrics')

    def reset(self):
        '''Reset the desired class data'''
        raise NotImplementedError('Metrics class must be able to reset')
