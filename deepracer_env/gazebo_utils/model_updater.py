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
"""Singleton helper for driving Gazebo model pose and visual updates."""

import logging

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelProperties, GetModelPropertiesRequest, SetModelState, SetModelStateRequest
from std_msgs.msg import ColorRGBA

from deepracer_msgs.srv import (
    GetVisualNames, GetVisualNamesRequest,
    GetVisuals, GetVisualsRequest,
    SetVisualColors, SetVisualColorsRequest,
    SetVisualTransparencies, SetVisualTransparenciesRequest,
)

from deepracer_env.domain_randomizations.constants import GazeboServiceName
from deepracer_env.rospy_wrappers import ServiceProxyWrapper
from deepracer_env.track_geom.constants import SET_MODEL_STATE
from deepracer_env.log_handler.logger import Logger

logger = Logger(__name__, logging.INFO).get_logger()

# Colour name → (r, g, b, a) mapping used by update_color()
_COLOUR_MAP = {
    "black":   (0.0,  0.0,  0.0,  1.0),
    "white":   (1.0,  1.0,  1.0,  1.0),
    "grey":    (0.5,  0.5,  0.5,  1.0),
    "gray":    (0.5,  0.5,  0.5,  1.0),
    "red":     (1.0,  0.0,  0.0,  1.0),
    "blue":    (0.0,  0.0,  1.0,  1.0),
    "green":   (0.0,  0.8,  0.0,  1.0),
    "orange":  (1.0,  0.5,  0.0,  1.0),
    "purple":  (0.5,  0.0,  0.5,  1.0),
    "pink":    (1.0,  0.41, 0.71, 1.0),
    "cyan":    (0.0,  1.0,  1.0,  1.0),
    "yellow":  (1.0,  1.0,  0.0,  1.0),
}


def _colour_rgba(colour_name: str) -> ColorRGBA:
    """Return a ColorRGBA for *colour_name*.  Falls back to black if unknown."""
    r, g, b, a = _COLOUR_MAP.get(colour_name.lower(), (0.0, 0.0, 0.0, 1.0))
    return ColorRGBA(r=r, g=g, b=b, a=a)


class ModelUpdater:
    """Singleton that wraps the Gazebo ROS services for pose and visual updates."""

    _instance = None

    def __init__(self):
        self._set_model_state = ServiceProxyWrapper(SET_MODEL_STATE, SetModelState)
        self._get_model_prop = ServiceProxyWrapper(
            GazeboServiceName.GET_MODEL_PROPERTIES.value, GetModelProperties)
        self._get_visual_names = ServiceProxyWrapper(
            GazeboServiceName.GET_VISUAL_NAMES.value, GetVisualNames)
        self._get_visuals = ServiceProxyWrapper(
            GazeboServiceName.GET_VISUALS.value, GetVisuals)
        self._set_visual_colors = ServiceProxyWrapper(
            GazeboServiceName.SET_VISUAL_COLORS.value, SetVisualColors)
        self._set_visual_transparencies = ServiceProxyWrapper(
            GazeboServiceName.SET_VISUAL_TRANSPARENCIES.value, SetVisualTransparencies)

    @classmethod
    def get_instance(cls) -> "ModelUpdater":
        """Return (creating if necessary) the singleton instance."""
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_model_pose(self, model_name: str, model_pose) -> None:
        """Teleport *model_name* to *model_pose* (geometry_msgs/Pose)."""
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = model_pose
        model_state.reference_frame = "world"
        resp = self._set_model_state(SetModelStateRequest(model_state=model_state))
        if not resp.success:
            logger.warning("set_model_pose failed for %s: %s", model_name, resp.status_message)

    def get_model_visuals(self, racecar_name: str):
        """Return a GetVisuals response containing all visuals for *racecar_name*.

        The returned object has ``link_names``, ``visual_names``, ``ambients``,
        ``diffuses``, ``transparencies``, etc. — as defined in GetVisuals.srv.
        """
        # 1. Get the list of link names for this model.
        prop_resp = self._get_model_prop(GetModelPropertiesRequest(model_name=racecar_name))
        link_names = list(prop_resp.body_names)

        # 2. Resolve which visual belongs to each link.
        vis_name_resp = self._get_visual_names(
            GetVisualNamesRequest(link_names=link_names))
        visual_names = list(vis_name_resp.visual_names)
        # Filter out empty entries returned for links with no visuals.
        filtered_links = []
        filtered_visuals = []
        for ln, vn in zip(link_names, visual_names):
            if vn:
                filtered_links.append(ln)
                filtered_visuals.append(vn)

        # 3. Fetch the full visual data.
        visuals_resp = self._get_visuals(
            GetVisualsRequest(link_names=filtered_links, visual_names=filtered_visuals))
        return visuals_resp

    def hide_visuals(self, visuals, ignore_keywords=None) -> None:
        """Make all visuals fully transparent except those whose name contains a
        keyword in *ignore_keywords*.

        Parameters
        ----------
        visuals:
            GetVisuals response returned by :meth:`get_model_visuals`.
        ignore_keywords:
            List of strings; visuals whose ``visual_name`` contains any of these
            strings are left unchanged.
        """
        if ignore_keywords is None:
            ignore_keywords = []

        hide_links = []
        hide_visuals_list = []
        for ln, vn in zip(visuals.link_names, visuals.visual_names):
            if not any(kw in vn for kw in ignore_keywords):
                hide_links.append(ln)
                hide_visuals_list.append(vn)

        if not hide_links:
            return

        req = SetVisualTransparenciesRequest(
            link_names=hide_links,
            visual_names=hide_visuals_list,
            transparencies=[1.0] * len(hide_links),
            block=True,
        )
        resp = self._set_visual_transparencies(req)
        if not resp.success:
            logger.warning("hide_visuals failed: %s", resp.status_message)

    def update_color(self, visuals, color: str) -> None:
        """Apply *color* to all visuals (ambient + diffuse channels).

        Parameters
        ----------
        visuals:
            GetVisuals response returned by :meth:`get_model_visuals`.
        color:
            Colour name string, e.g. ``"Black"``, ``"Red"``.
        """
        rgba = _colour_rgba(color)
        n = len(visuals.link_names)
        req = SetVisualColorsRequest(
            link_names=list(visuals.link_names),
            visual_names=list(visuals.visual_names),
            ambients=[rgba] * n,
            diffuses=[rgba] * n,
            speculars=[ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)] * n,
            emissives=[ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)] * n,
            block=True,
        )
        resp = self._set_visual_colors(req)
        if not resp.success:
            logger.warning("update_color failed: %s", resp.status_message)
