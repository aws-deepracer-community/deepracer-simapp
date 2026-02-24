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

'''This module should be used for common classes and methods that can
   be used in any python module.
'''
import abc
from typing import Any

class ObserverInterface(metaclass=abc.ABCMeta):
    '''This class defines the interface for an observer, which can be registered
       to a sink and can receive notifications.
    '''
    def update(self, data: Any) -> None:
        '''Updates the observer with the data sent from the sink
           data - Data received from the sink
        '''
        raise NotImplementedError('Observer must implement update method')
