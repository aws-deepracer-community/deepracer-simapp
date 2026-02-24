#!/usr/bin/env python3
"""Minimal example: custom agent training with DeepRacerEnv.

DeepRacerEnv handles all ROS/Gazebo wiring internally — you only need to
provide a reward function, then optionally customise sensors or controller
parameters.

Prerequisites
-------------
* A running Gazebo + DeepRacer ROS stack
  (``roslaunch deepracer_simulation deepracer_rl.launch``).
* Python packages: ``gymnasium``, ``stable-baselines3``.
"""
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

from markov.environments.deepracer_env import DeepRacerEnv
from markov.sensors.constants import Input
import markov.agent_ctrl.constants as ctrl_const


def reward_function(params: dict) -> float:
    """Maximise forward progress while staying on track.

    ``params`` contains all reward param keys defined in
    ``markov.agent_ctrl.constants.RewardParam``.
    """
    if not params['all_wheels_on_track']:
        return 1e-3

    return float(max(params['progress'] * params['speed'] / 4.0, 1e-3))

    


if __name__ == '__main__':
    env = DeepRacerEnv(reward_fn=reward_function)

    # ── Optional customisation examples (uncomment to use) ────────────────────
    # Add a LIDAR sensor alongside the camera:
    # env = DeepRacerEnv(
    #     reward_fn=reward_function,
    #     sensors=[Input.CAMERA.value, Input.LIDAR.value],
    # )
    #
    # Override individual controller parameters:
    # env = DeepRacerEnv(
    #     reward_fn=reward_function,
    #     config={
    #         ctrl_const.ConfigParams.NUMBER_OF_RESETS.value: 0,
    #         ctrl_const.ConfigParams.COLLISION_PENALTY.value: 5.0,
    #     },
    # )

    # Optional: verify the environment conforms to the gymnasium API.
    check_env(env, warn=True)

    # ── Train with PPO ────────────────────────────────────────────────────────
    model = PPO(
        policy='MultiInputPolicy',  # handles Dict observation spaces
        env=env,
        verbose=1,
        n_steps=256,
        batch_size=64,
        learning_rate=3e-4,
        ent_coef=0.01,
        tensorboard_log='./tb_logs/',
    )

    model.learn(total_timesteps=50_000)
    model.save('deepracer_ppo')
    print('Model saved to deepracer_ppo.zip')

    env.close()

