'''Gymnasium environment for the AWS DeepRacer Gazebo simulation.

Minimal usage
-------------
Pass a reward function and the environment builds everything else::

    from deepracer_env.environments.deepracer_env import DeepRacerEnv

    def my_reward(params: dict) -> float:
        return float(params['progress']) * float(params['speed']) / 4.0

    env = DeepRacerEnv(reward_fn=my_reward)
    obs, info = env.reset()
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())

Customising sensors
-------------------
Pass a list of ``Input`` value strings::

    from deepracer_env.sensors.constants import Input

    env = DeepRacerEnv(
        reward_fn=my_reward,
        sensors=[Input.CAMERA.value, Input.LIDAR.value],
    )

Overriding controller parameters
---------------------------------
Provide a partial ``config`` dict with any :class:`~deepracer_env.agent_ctrl.constants.ConfigParams`
*value* keys you want to change; the rest keep their defaults::

    import deepracer_env.agent_ctrl.constants as ctrl_const

    env = DeepRacerEnv(
        reward_fn=my_reward,
        config={ctrl_const.ConfigParams.NUMBER_OF_RESETS.value: 0},
    )

Full control
------------
Build an :class:`~deepracer_env.agents.agent.Agent` yourself and pass it directly.
In this case ``reward_fn``, ``sensors``, and ``config`` are ignored::

    from deepracer_env.agents.agent import Agent
    from deepracer_env.sensors.composite_sensor import CompositeSensor
    from deepracer_env.agent_ctrl.rollout_agent_ctrl import RolloutCtrl

    sensor = CompositeSensor()
    ctrl   = RolloutCtrl(my_config, my_metrics, is_training=True)
    env    = DeepRacerEnv(agent=Agent(sensor, ctrl))

Action space
------------
A continuous ``Box(2,)`` vector: ``[steering_angle_deg, speed_m_s]``.

* ``steering_angle_deg`` ∈ [−30, 30] — positive turns the car left.
* ``speed_m_s`` ∈ [0.1, 4.0]  — forward speed in m/s.

Observation space
-----------------
A ``gymnasium.spaces.Dict`` whose keys are the active sensor ``Input.value``
strings (e.g. ``"CAMERA"``, ``"LIDAR"``).
Each value is a ``Box`` matching the sensor's output shape.
'''
import logging
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import gymnasium

import deepracer_env.agent_ctrl.constants as ctrl_const
from deepracer_env.environments.constants import (
    LINK_NAMES,
    VELOCITY_TOPICS,
    STEERING_TOPICS,
)
from deepracer_env.sensors.constants import Input
from deepracer_env.constants import SIMAPP_VERSION_5
from deepracer_env.log_handler.logger import Logger

LOG = Logger(__name__, logging.INFO).get_logger()

# ---------------------------------------------------------------------------
# Default action space: [steering_angle_deg, speed_m_s]
# ---------------------------------------------------------------------------
DEFAULT_ACTION_SPACE: gymnasium.spaces.Box = gymnasium.spaces.Box(
    low=np.array([-30.0, 0.1], dtype=np.float32),
    high=np.array([30.0, 4.0], dtype=np.float32),
    dtype=np.float32,
)

# ---------------------------------------------------------------------------
# Default sensors
# ---------------------------------------------------------------------------
DEFAULT_SENSORS: List[str] = [Input.CAMERA.value]

# ---------------------------------------------------------------------------
# Default controller config
# ---------------------------------------------------------------------------
_DEFAULT_CTRL_CONFIG: Dict[str, Any] = {
    ctrl_const.ConfigParams.AGENT_NAME.value:               'racecar',
    ctrl_const.ConfigParams.LINK_NAME_LIST.value:           LINK_NAMES,
    ctrl_const.ConfigParams.VELOCITY_LIST.value:            VELOCITY_TOPICS,
    ctrl_const.ConfigParams.STEERING_LIST.value:            STEERING_TOPICS,
    ctrl_const.ConfigParams.ACTION_SPACE.value:             DEFAULT_ACTION_SPACE,
    ctrl_const.ConfigParams.VERSION.value:                  SIMAPP_VERSION_5,
    ctrl_const.ConfigParams.IS_CONTINUOUS.value:            True,
    ctrl_const.ConfigParams.NUMBER_OF_RESETS.value:         5,
    ctrl_const.ConfigParams.PENALTY_SECONDS.value:          2.0,
    ctrl_const.ConfigParams.NUMBER_OF_TRIALS.value:         1000,
    ctrl_const.ConfigParams.RACE_TYPE.value:                'TIME_TRIAL',
    ctrl_const.ConfigParams.COLLISION_PENALTY.value:        2.0,
    ctrl_const.ConfigParams.OFF_TRACK_PENALTY.value:        2.0,
    ctrl_const.ConfigParams.IMMOBILIZED_PENALTY.value:      2.0,
    ctrl_const.ConfigParams.REVERSE_PENALTY.value:          2.0,
    ctrl_const.ConfigParams.CHANGE_START.value:             True,
    ctrl_const.ConfigParams.ALT_DIR.value:                  False,
    ctrl_const.ConfigParams.ROUND_ROBIN_ADVANCE_DIST.value: 0.05,
    ctrl_const.ConfigParams.START_POSITION_OFFSET.value:    0.0,
    ctrl_const.ConfigParams.DONE_CONDITION.value:           any,
    ctrl_const.ConfigParams.PARK_POSITIONS.value:           [(0.0, 0.0, 0.0)],
}


class _NoopMetrics:
    '''No-op metrics implementation used when no metrics backend is provided.'''
    def reset(self):                         pass
    def append_episode_metrics(self, **kw):  pass
    def upload_episode_metrics(self):        pass
    def upload_step_metrics(self, _):        pass
    def update_mp4_video_metrics(self, _):   pass


def _build_agent(
    reward_fn: Callable[[dict], float],
    sensors: List[str],
    config: Optional[Dict[str, Any]],
    is_training: bool,
):
    '''Build a default :class:`~deepracer_env.agents.agent.Agent` from its component parts.

    This is the wiring that happens inside ROS/Gazebo — hidden from the user
    unless they need to customise it.
    '''
    # Initialise the ROS node if one hasn't been started yet.
    import rospy
    if not rospy.core.is_initialized():
        rospy.init_node('deepracer_env', anonymous=True)

    # Deferred imports keep the top-level import cost low and avoid
    # circular dependencies at module load time.
    from deepracer_env.agents.agent import Agent
    from deepracer_env.sensors.composite_sensor import CompositeSensor
    from deepracer_env.sensors.sensors_rollout import SensorFactory
    from deepracer_env.agent_ctrl.rollout_agent_ctrl import RolloutCtrl

    racecar_name = 'racecar'

    # Build composite sensor
    composite_sensor = CompositeSensor()
    for sensor_type in sensors:
        composite_sensor.add_sensor(
            SensorFactory.create_sensor(racecar_name, sensor_type, {})
        )

    # Merge user overrides into the default config
    ctrl_config = {**_DEFAULT_CTRL_CONFIG}
    ctrl_config[ctrl_const.ConfigParams.REWARD.value] = reward_fn
    if config:
        ctrl_config.update(config)

    ctrl = RolloutCtrl(ctrl_config, metrics=_NoopMetrics(), is_training=is_training)
    return Agent(composite_sensor, ctrl)


class DeepRacerEnv(gymnasium.Env):
    '''Gymnasium-compatible environment wrapping the DeepRacer Gazebo simulator.

    See the module docstring for usage examples.

    Args:
        reward_fn: Callable ``(params: dict) -> float``.  Required unless
            ``agent`` is provided.  ``params`` contains all keys from
            :class:`~deepracer_env.agent_ctrl.constants.RewardParam`.
        sensors: Active sensor types as ``Input.value`` strings.
            Defaults to :data:`DEFAULT_SENSORS` (``["CAMERA"]``).
        config: Partial override dict for the controller.  Keys are the
            string *values* of :class:`~deepracer_env.agent_ctrl.constants.ConfigParams`.
            Any key not present keeps its default value.
        agent: Fully-initialised :class:`~deepracer_env.agents.agent.Agent`.
            When provided, ``reward_fn``, ``sensors``, and ``config`` are
            ignored — you are in full control.
        action_space: Override the action space.  Defaults to
            :data:`DEFAULT_ACTION_SPACE`.
        is_training: Passed to the default controller; controls whether
            the start position advances between episodes.  Ignored when
            ``agent`` is provided explicitly.
    '''

    metadata = {'render_modes': []}

    def __init__(
        self,
        reward_fn: Optional[Callable[[dict], float]] = None,
        sensors: Optional[List[str]] = None,
        config: Optional[Dict[str, Any]] = None,
        agent=None,
        action_space: Optional[gymnasium.spaces.Space] = None,
        is_training: bool = True,
    ) -> None:
        super().__init__()

        if agent is not None:
            self._agent = agent
        else:
            if reward_fn is None:
                raise ValueError(
                    'Provide either reward_fn (to use the default agent) '
                    'or agent (to supply your own).'
                )
            self._agent = _build_agent(
                reward_fn=reward_fn,
                sensors=sensors if sensors is not None else DEFAULT_SENSORS,
                config=config,
                is_training=is_training,
            )

        self.action_space: gymnasium.spaces.Space = (
            action_space if action_space is not None else DEFAULT_ACTION_SPACE
        )
        obs_space = self._agent.get_observation_space()
        if obs_space is None:
            raise ValueError(
                'Agent has no sensor configured — cannot determine observation_space.'
            )
        self.observation_space: gymnasium.spaces.Dict = obs_space

        # Store last step info for diagnostics
        self._last_step_info: Dict[str, Any] = {}

    # ------------------------------------------------------------------
    # Core gymnasium.Env interface
    # ------------------------------------------------------------------

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None,
    ) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        '''Reset the environment to the beginning of a new episode.

        Returns:
            observation (dict): Initial sensor observation.
            info (dict): Auxiliary diagnostic information (empty by default).
        '''
        super().reset(seed=seed)
        LOG.debug('DeepRacerEnv.reset() called')
        obs = self._agent.reset_agent()
        self._last_step_info = {}
        return obs, {}

    def step(
        self, action: np.ndarray
    ) -> Tuple[Dict[str, np.ndarray], float, bool, bool, Dict[str, Any]]:
        '''Run one environment step.

        Publishes *action* to Gazebo via ROS, waits for the next sensor
        observation, and returns the standard gymnasium 5-tuple.

        Args:
            action (np.ndarray): ``[steering_angle_deg, speed_m_s]``.

        Returns:
            observation (dict): Sensor observation after the action.
            reward (float): Scalar reward from the reward function.
            terminated (bool): ``True`` if the episode ended naturally
                (e.g. lap complete, off-track, time-up).
            truncated (bool): Always ``False`` (no external time limit
                beyond what the reset-rules manager handles).
            info (dict): Step metrics forwarded from the controller.
        '''
        # 1. Publish command to Gazebo
        self._agent.send_action(action)
        # 2. Advance internal state (read car position, compute metrics)
        agents_info_map = self._agent.update_agent(action)
        # 3. Evaluate the action (compute reward, termination)
        obs, reward, done = self._agent.judge_action(action, agents_info_map)
        self._last_step_info = agents_info_map
        return obs, float(reward), bool(done), False, agents_info_map

    def close(self) -> None:
        '''Stop the car and release resources.'''
        LOG.info('DeepRacerEnv.close() — setting car speed to zero')
        try:
            self._agent.send_action(np.array([0.0, 0.0], dtype=np.float32))
        except Exception:
            pass

    def render(self) -> None:  # type: ignore[override]
        '''Rendering is handled externally by Gazebo / RViz.'''
        pass

    # ------------------------------------------------------------------
    # Convenience helpers
    # ------------------------------------------------------------------

    @property
    def agent(self):
        '''The underlying :class:`~deepracer_env.agents.agent.Agent` instance.'''
        return self._agent

    @property
    def last_step_info(self) -> Dict[str, Any]:
        '''Diagnostic info dict from the most recent :meth:`step` call.'''
        return self._last_step_info
