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

import abc

# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})


class AbstractRandomizer(ABC):
    """
    Abstract Randomizer class
    """
    def __init__(self):
        pass

    def randomize(self):
        self._randomize()

    @abc.abstractmethod
    def _randomize(self):
        """
        Randomize
        """
        raise NotImplementedError('Randomizer must implement this function')
