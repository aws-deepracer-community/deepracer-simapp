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

''' This module hauses all the constants used by the AWS DeepRacer enviroment
    package.
'''
# Dimensions of the input training image
TRAINING_IMAGE_SIZE = (160, 120)
# Dimensions of input lidar data
TRAINING_LIDAR_SIZE = 64

# SECTOR_LIDAR sensor constants
# Number of lidar sectors - to be used when filtering in observation_binary_filter
NUMBER_OF_LIDAR_SECTORS = 8
# Max clipping distance for sector lidar sensor
SECTOR_LIDAR_CLIPPING_DIST = 0.5

LINK_NAMES = ['racecar::left_rear_wheel', 'racecar::left_front_wheel',
              'racecar::right_rear_wheel', 'racecar::right_front_wheel']

# List of required velocity topics, one topic per wheel
VELOCITY_TOPICS = ['/racecar/left_rear_wheel_velocity_controller/command',
                   '/racecar/right_rear_wheel_velocity_controller/command',
                   '/racecar/left_front_wheel_velocity_controller/command',
                   '/racecar/right_front_wheel_velocity_controller/command']

# List of required steering hinges
STEERING_TOPICS = ['/racecar/left_steering_hinge_position_controller/command',
                   '/racecar/right_steering_hinge_position_controller/command']

# List of all effort joints
EFFORT_JOINTS = ['/racecar/left_rear_wheel_joint', '/racecar/right_rear_wheel_joint',
                 '/racecar/left_front_wheel_joint', '/racecar/right_front_wheel_joint',
                 '/racecar/left_steering_hinge_joint', '/racecar/right_steering_hinge_joint']

# The number of steps to wait before checking if the car is stuck
# This number should correspond to the camera FPS, since it is pacing the
# step rate.
NUM_STEPS_TO_CHECK_STUCK = 15

#Defines to upload SIM_TRACE data in S3
TRAINING_SIMTRACE_DATA_S3_OBJECT_KEY = "sim_inference_logs/TrainingSimTraceData.csv"
EVALUATION_SIMTRACE_DATA_S3_OBJECT_KEY = "sim_inference_logs/EvaluationSimTraceData.csv"
SIMAPP_DATA_UPLOAD_TIME_TO_S3 = 60 #in seconds
