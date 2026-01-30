# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from enum import Enum


class GeometryType(Enum):
    """Geometry Type"""
    BOX = 1
    CYLINDER = 2
    SPHERE = 3
    PLANE = 4
    IMAGE = 5
    HEIGHTMAP = 6
    MESH = 7
    TRIANGLE_FAN = 8
    LINE_STRIP = 9
    POLYLINE = 10
    EMPTY = 11


class GazeboServiceName(Enum):
    """Gazebo Service Names"""
    GET_MODEL_PROPERTIES = '/deepracer/get_model_properties'
    GET_LIGHT_NAMES = '/deepracer/get_light_names'
    GET_VISUAL_NAMES = '/deepracer/get_visual_names'
    GET_VISUAL = '/deepracer/get_visual'
    GET_VISUALS = '/deepracer/get_visuals'
    SET_LIGHT_PROPERTIES = '/deepracer/set_light_properties'
    SET_VISUAL_COLOR = '/deepracer/set_visual_color'
    SET_VISUAL_COLORS = '/deepracer/set_visual_colors'
    SET_VISUAL_TRANSPARENCY = '/deepracer/set_visual_transparency'
    SET_VISUAL_TRANSPARENCIES = '/deepracer/set_visual_transparencies'
    SET_VISUAL_VISIBLE = '/deepracer/set_visual_visible'
    SET_VISUAL_VISIBLES = '/deepracer/set_visual_visibles'
    SET_VISUAL_POSE = '/deepracer/set_visual_pose'
    SET_VISUAL_POSES = '/deepracer/set_visual_poses'
    SET_VISUAL_MESH = '/deepracer/set_visual_mesh'
    SET_VISUAL_MESHES = '/deepracer/set_visual_meshes'
    PAUSE_PHYSICS = '/deepracer/pause_physics_dr'
    UNPAUSE_PHYSICS = '/deepracer/unpause_physics_dr'


class ModelRandomizerType(Enum):
    """ Model Randomizer Type

    MODEL type will randomize the color of overall model.
    LINK type will randomize the color for each link.
    VISUAL type will randomize the color for each link's visual
    """
    MODEL = "model"
    LINK = "link"
    VISUAL = "visual"


class RangeType(Enum):
    """Range Type"""
    COLOR = 'color'
    ATTENUATION = 'attenuation'


RANGE_MIN = 'min'
RANGE_MAX = 'max'


class Color(Enum):
    """Color attributes"""
    R = 'r'
    G = 'g'
    B = 'b'


class Attenuation(Enum):
    """Light attenuation attributes"""
    CONSTANT = 'constant'
    LINEAR = 'linear'
    QUADRATIC = 'quadratic'


