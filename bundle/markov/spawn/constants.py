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

"""constants for spawn"""


class DeepRacerPackages():
    """DeepRacer ROS Packages
    """
    DEEPRACER_SIMULATION_ENVIRONMENT = \
        "deepracer_simulation_environment"


# LIDAR values: these values are defined in racecar.launch
# if you changed these value, please make sure you also update
# racecar.launch lidar input value
LIDAR_360_DEGREE_SAMPLE = "64"
LIDAR_360_DEGREE_HORIZONTAL_RESOLUTION = "1"
LIDAR_360_DEGREE_MIN_ANGLE = "-2.61799"
LIDAR_360_DEGREE_MAX_ANGLE = "2.61799"
LIDAR_360_DEGREE_MIN_RANGE = "0.15"
LIDAR_360_DEGREE_MAX_RANGE = "0.5"
LIDAR_360_DEGREE_RANGE_RESOLUTION = "0.01"
LIDAR_360_DEGREE_NOISE_MEAN = "0.0"
LIDAR_360_DEGREE_NOISE_STDDEV = "0.01"

# sleep seconds after delete/spawn gazebo service call
SLEEP_SECONDS_AFTER_GAZEBO_MODEL_SERVICE_CALL = 5
