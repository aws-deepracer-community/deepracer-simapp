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

import time
import rospy
import markov.rollout_constants as const
from markov.log_handler.deepracer_exceptions import GenericRolloutException
from markov.domain_randomizations.constants import GazeboServiceName
from markov.rospy_wrappers import ServiceProxyWrapper
from markov.track_geom.constants import SET_MODEL_STATE
from markov.track_geom.utils import euler_to_quaternion
from markov.gazebo_tracker.trackers.get_model_state_tracker import GetModelStateTracker
from markov.gazebo_tracker.trackers.set_model_state_tracker import SetModelStateTracker
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelProperties, GetModelPropertiesRequest
from deepracer_msgs.srv import (GetVisualNames, GetVisualNamesRequest,
                                GetVisuals, GetVisualsRequest,
                                SetVisualColors, SetVisualColorsRequest,
                                SetVisualTransparencies, SetVisualTransparenciesRequest,
                                SetVisualVisibles, SetVisualVisiblesRequest)

GAZEBO_SERVICES = ["PAUSE_PHYSICS", "UNPAUSE_PHYSICS",
                   "GET_MODEL_PROPERTIES",
                   "GET_VISUAL_NAMES",
                   "GET_VISUALS",
                   "SET_VISUAL_COLORS",
                   "SET_VISUAL_TRANSPARENCIES",
                   "SET_VISUAL_VISIBLES"]


class ModelUpdater(object):
    """
    ModelUpdater class
    """
    _instance_ = None

    @staticmethod
    def get_instance():
        """Method for getting a reference to the Model Updater object"""
        if ModelUpdater._instance_ is None:
            ModelUpdater()
        return ModelUpdater._instance_

    def __init__(self):
        """Initialize a ModelUpdater Object.

        Raises:
            GenericRolloutException: raise a GenericRolloutException if the object is no longer singleton.
        """
        if ModelUpdater._instance_ is not None:
            raise GenericRolloutException("Attempting to construct multiple ModelUpdater")

        # Wait for required services to be available
        rospy.wait_for_service(SET_MODEL_STATE)
        # Wait for gazebo plugin services to be available
        for service in GazeboServiceName:
            if service.name in GAZEBO_SERVICES:
                rospy.wait_for_service(service.value)
        # Gazebo service that allows us to position the car
        self._model_state_client = ServiceProxyWrapper(SET_MODEL_STATE, SetModelState)

        self._get_model_prop = ServiceProxyWrapper(GazeboServiceName.GET_MODEL_PROPERTIES.value,
                                                   GetModelProperties)
        self._get_visual_names = ServiceProxyWrapper(GazeboServiceName.GET_VISUAL_NAMES.value,
                                                     GetVisualNames)
        self._get_visuals = ServiceProxyWrapper(GazeboServiceName.GET_VISUALS.value, GetVisuals)
        self._set_visual_colors = ServiceProxyWrapper(GazeboServiceName.SET_VISUAL_COLORS.value,
                                                      SetVisualColors)
        self._set_visual_visibles = ServiceProxyWrapper(GazeboServiceName.SET_VISUAL_VISIBLES.value,
                                                        SetVisualVisibles)
        self._set_visual_transparencies = ServiceProxyWrapper(GazeboServiceName.SET_VISUAL_TRANSPARENCIES.value,
                                                              SetVisualTransparencies)
        self._pause_physics = ServiceProxyWrapper(GazeboServiceName.PAUSE_PHYSICS.value, Empty)
        self._unpause_physics = ServiceProxyWrapper(GazeboServiceName.UNPAUSE_PHYSICS.value, Empty)
        self._set_model_state_tracker = SetModelStateTracker.get_instance()
        self._get_model_state_tracker = GetModelStateTracker.get_instance()
        # there should be only one model updater instance
        ModelUpdater._instance_ = self

    @property
    def pause_physics_service(self):
        """return the gazebo service for pause physics.

        Returns:
            ServiceProxyWrapper: The pause physics gazebo service
        """
        return self._pause_physics

    @property
    def unpause_physics_service(self):
        """return the gazebo service for unpause physics.

        Returns:
            ServiceProxyWrapper: The unpause physics gazebo service
        """
        return self._unpause_physics

    def update_model_color(self, model_name, car_color):
        """ Update the model's color.

        Args:
            model_name (str): The model name for the race car we want to update color for.
                              e.g. racecar_0
            car_color (str): The car color we want to update the visuals to.
                             e.g. Purple, Orange, White, Black, etc.
        """
        visuals = self.get_model_visuals(model_name)

        self.update_color(visuals=visuals,
                          car_color=car_color)

    def update_color(self, visuals, car_color):
        """Update the model's color using it's visuals

        Args:
            visuals (Visuals): The visuals of the current model
            car_color (str): The car color we want to update the visuals to.
                             e.g. Purple, Orange, White, Black, etc.
        """
        link_names = []
        visual_names = []
        ambients, diffuses, speculars, emissives = [], [], [], []

        for visual_name, link_name in zip(visuals.visual_names, visuals.link_names):
            if "car_body_link" in visual_name:
                visual_names.append(visual_name)
                link_names.append(link_name)
                ambient = ColorRGBA(const.COLOR_MAP[car_color].r * 0.1,
                                    const.COLOR_MAP[car_color].g * 0.1,
                                    const.COLOR_MAP[car_color].b * 0.1,
                                    const.COLOR_MAP[car_color].a)
                diffuse = ColorRGBA(const.COLOR_MAP[car_color].r * 0.35,
                                    const.COLOR_MAP[car_color].g * 0.35,
                                    const.COLOR_MAP[car_color].b * 0.35,
                                    const.COLOR_MAP[car_color].a)

                ambients.append(ambient)
                diffuses.append(diffuse)
                speculars.append(const.DEFAULT_COLOR)
                emissives.append(const.DEFAULT_COLOR)
        if len(visual_names) > 0:
            req = SetVisualColorsRequest()
            req.visual_names = visual_names
            req.link_names = link_names
            req.ambients = ambients
            req.diffuses = diffuses
            req.speculars = speculars
            req.emissives = emissives
            self._set_visual_colors(req)

    def get_model_visuals(self, model_name):
        """Get the model visuals associated to the model name

        Args:
            model_name (str): The model name for the race car we want to hide visuals for.
                              e.g. racecar_0

        Returns:
            Visuals: The visuals of the current model.
        """
        # Get all model's link names
        body_names = self._get_model_prop(GetModelPropertiesRequest(model_name=model_name)) \
            .body_names
        link_names = ["%s::%s" % (model_name, b) for b in body_names]
        res = self._get_visual_names(GetVisualNamesRequest(link_names=link_names))
        get_visuals_req = GetVisualsRequest(link_names=res.link_names,
                                            visual_names=res.visual_names)
        visuals = self._get_visuals(get_visuals_req)

        return visuals

    def hide_visuals(self, visuals, ignore_keywords, retry_num=3):
        """
        Set the transparencies to 1.0 for all links and visibles to False
        to hide the model's visuals passed in.

        Args:
            visuals (Visuals): the visuals for the current model.
            ignore_keywords (List): list of keywords name in visual which
            we should not hidden its visual
            retry_num (int): number of retries for hiding visuals.
        """
        link_names = []
        visual_names = []

        for visual_name, link_name in zip(visuals.visual_names, visuals.link_names):
            is_to_hide = True
            for name in ignore_keywords:
                if name in visual_name:
                    is_to_hide = False
            if is_to_hide:
                visual_names.append(visual_name)
                link_names.append(link_name)

        retry_count = 0
        while not self._are_visuals_hidden(link_names, visual_names):
            if retry_count >= retry_num:
                raise GenericRolloutException("Hide Visuals Gazebo Services constantly failing. Something Wrong with gzserver.")
            self._set_visible_false(link_names, visual_names)
            self._set_transparent(link_names, visual_names)
            retry_count += 1
            time.sleep(2)  # sleep 2 seconds until next retry

    def _set_visible_false(self, link_names, visual_names, block=True):
        """Set visuals of the passed in link_names and visual_names as not visible.

        Args:
            link_names ([str]): A list of link names.
            visual_names ([str]): A list of visual names.
            block (bool, optional): Is the call is blocking. Defaults to True.
        """
        req = SetVisualVisiblesRequest()
        req.link_names = link_names
        req.visual_names = visual_names
        req.visibles = [False] * len(link_names)
        req.block = block
        self._set_visual_visibles(req)

    def _set_transparent(self, link_names, visual_names, block=True):
        """Set transparencies of the passed in link_names and visual_names as transparent.

        Args:
            link_names ([str]): A list of link names.
            visual_names ([str]): A list of visual names.
            block (bool, optional): Is the call is blocking. Defaults to True.
        """
        req = SetVisualTransparenciesRequest()
        req.link_names = link_names
        req.visual_names = visual_names
        req.transparencies = [1.0] * len(link_names)
        req.block = block
        self._set_visual_transparencies(req)

    def _are_visuals_hidden(self, link_names, visual_names):
        """Check if the visuals are hidden.
           If all the visuals are visible as False or all are transparent,
           return True.

        Args:
            link_names ([str]): A list of link names.
            visual_names ([str]): A list of visual names.

        Returns:
            bool: If the visuals are hiddens
        """
        get_visuals_req = GetVisualsRequest(link_names=link_names,
                                            visual_names=visual_names)
        visuals = self._get_visuals(get_visuals_req)
        # all transparency are 1.0
        all_transparent = (int(sum(visuals.transparencies)) == len(link_names))
        # all visible are false
        all_non_visible = (sum(visuals.visibles) == 0)
        return all_transparent or all_non_visible

    def _construct_model_pose(self, model_position, yaw):
        car_pose = Pose()
        orientation = euler_to_quaternion(yaw=yaw)
        car_pose.position.x = model_position[0]
        car_pose.position.y = model_position[1]
        car_pose.position.z = 0.0
        car_pose.orientation.x = orientation[0]
        car_pose.orientation.y = orientation[1]
        car_pose.orientation.z = orientation[2]
        car_pose.orientation.w = orientation[3]
        return car_pose

    def set_model_position(self, model_name, model_position, yaw, is_blocking=False):
        ''' get initial car position on the track
        '''
        model_pose = self._construct_model_pose(model_position, yaw)
        return self.set_model_pose(model_name, model_pose, is_blocking)

    def set_model_pose(self, model_name, model_pose, is_blocking=False):
        """Set the model state with a model pose.

        Args:
            model_name (str): The model name of the model state we want to set.
            model_pose (Pose): The pose we want to set the model to.

        Returns:
            [type]: [description]
        """
        # Construct the model state and send to Gazebo
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = model_pose
        model_state.twist.linear.x = 0
        model_state.twist.linear.y = 0
        model_state.twist.linear.z = 0
        model_state.twist.angular.x = 0
        model_state.twist.angular.y = 0
        model_state.twist.angular.z = 0
        return self.set_model_state(model_state, is_blocking)

    def set_model_state(self, model_state, is_blocking=False):
        """Set the models state in gazebo.

        Args:
            model_state (ModelState): The current model state to set to.

        Returns:
            ModelState: The latest model state of the robot.
        """
        if is_blocking:
            self._set_model_state_tracker.set_model_state(model_state, blocking=True)
            self._get_model_state_tracker.get_model_state(model_state.model_name, '', blocking=True)
        else:
            self._model_state_client(model_state)
        return model_state

    def pause_physics(self):
        """
        Pause the current gazebo environment physics.
        """
        self._pause_physics(EmptyRequest())

    def unpause_physics(self):
        """
        Pause the current gazebo environment physics.
        """
        self._unpause_physics(EmptyRequest())
