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
"""Constants for Gazebo ROS service names exposed by the DeepRacer Gazebo system plugin."""

from enum import Enum


class GazeboServiceName(Enum):
    """ROS service names published by the deepracer_gazebo_system_plugin."""
    # Gazebo physics control
    PAUSE_PHYSICS = "/gazebo/pause_physics"
    UNPAUSE_PHYSICS = "/gazebo/unpause_physics"

    # Model introspection
    GET_MODEL_PROPERTIES = "/gazebo/get_model_properties"

    # Visual name queries
    GET_VISUAL_NAMES = "get_visual_names"
    GET_VISUAL = "get_visual"
    GET_VISUALS = "get_visuals"

    # Visual color services
    SET_VISUAL_COLOR = "set_visual_color"
    SET_VISUAL_COLORS = "set_visual_colors"

    # Visual transparency services
    SET_VISUAL_TRANSPARENCY = "set_visual_transparency"
    SET_VISUAL_TRANSPARENCIES = "set_visual_transparencies"

    # Visual visibility services
    SET_VISUAL_VISIBLE = "set_visual_visible"
    SET_VISUAL_VISIBLES = "set_visual_visibles"

    # Visual pose services
    SET_VISUAL_POSE = "set_visual_pose"
    SET_VISUAL_POSES = "set_visual_poses"

    # Visual mesh services
    SET_VISUAL_MESH = "set_visual_mesh"
    SET_VISUAL_MESHES = "set_visual_meshes"
