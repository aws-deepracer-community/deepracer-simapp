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

from markov.log_handler.deepracer_exceptions import GenericRolloutException


class RandomizerManager(object):
    """
    Randomizer Manager class that manages multiple randomizer
    """
    _instance_ = None

    @staticmethod
    def get_instance():
        """Method for getting a reference to the camera manager object"""
        if RandomizerManager._instance_ is None:
            RandomizerManager()
        return RandomizerManager._instance_

    def __init__(self):
        if RandomizerManager._instance_ is not None:
            raise GenericRolloutException("Attempting to construct multiple Randomizer Manager")

        self.randomizers = []

        # there should be only one randomizer manager instance
        RandomizerManager._instance_ = self

    def add(self, randomizer):
        self.randomizers.append(randomizer)

    def randomize(self):
        for randomizer in self.randomizers:
            randomizer.randomize()
