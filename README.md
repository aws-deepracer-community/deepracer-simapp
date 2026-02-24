# deepracer-env

A [Gymnasium](https://gymnasium.farama.org/) environment for the AWS DeepRacer
autonomous-driving simulation, powered by **ROS Noetic** and **Gazebo 11**.

Instead of training against a fixed network with a plug-in reward function, this
repository exposes the full simulation as a standard `gymnasium.Env` so you can
bring any RL algorithm or neural-network architecture you like.

## Repository structure

```
deepracer-env/
├── deepracer_env/          # pip-installable Python package
│   ├── __init__.py         # registers DeepRacer-v0 with Gymnasium
│   ├── environments/       # DeepRacerEnv (gymnasium.Env subclass)
│   ├── sensors/            # Camera, LIDAR, sector-LIDAR sensor drivers
│   ├── agents/             # Agent (sensor + controller bundle)
│   ├── agent_ctrl/         # RolloutCtrl — Gazebo/ROS action publisher
│   ├── track_geom/         # Track geometry and waypoint utilities
│   └── ...
├── bundle/                 # ROS/Gazebo workspace (meshes, worlds, URDF, routes)
│   ├── src/                # ROS colcon workspace source
│   ├── meshes/
│   ├── worlds/
│   └── urdf/
├── docker/                 # Dockerfiles and pip requirements for the container
├── examples/
│   └── train.py            # Minimal PPO training example (Stable-Baselines3)
├── sample-config/          # Example reward function, robomaker.env, etc.
├── pyproject.toml          # Package metadata and build config
├── requirements.txt        # Minimal runtime dependencies
└── VERSION
```

## Prerequisites

| Component | Version |
|-----------|---------|
| Ubuntu    | 20.04   |
| Python    | 3.8+    |
| ROS       | Noetic  |
| Gazebo    | 11      |

A running Gazebo + DeepRacer ROS stack is required at runtime:

```bash
roslaunch deepracer_simulation_environment deepracer_rl.launch
```

## Installation

```bash
pip install -e .
# or, to also install training dependencies:
pip install -e .[examples]
```

## Quick start

```python
import gymnasium
import deepracer_env  # registers DeepRacer-v0

def my_reward(params: dict) -> float:
    if not params["all_wheels_on_track"]:
        return 1e-3
    return float(params["progress"] * params["speed"] / 4.0)

# Minimal — single camera sensor, all defaults
env = gymnasium.make("DeepRacer-v0", reward_fn=my_reward)

# Or construct directly for more control
from deepracer_env.environments.deepracer_env import DeepRacerEnv
from deepracer_env.sensors.constants import Input
import deepracer_env.agent_ctrl.constants as ctrl_const

env = DeepRacerEnv(
    reward_fn=my_reward,
    sensors=[Input.CAMERA.value, Input.LIDAR.value],
    config={ctrl_const.ConfigParams.COLLISION_PENALTY.value: 5.0},
)

obs, info = env.reset()
obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
```

See [examples/train.py](examples/train.py) for a complete Stable-Baselines3 training loop.

## Action space

`Box([-30, 0.1], [30, 4.0], dtype=float32)` — `[steering_angle_deg, speed_m_s]`

## Observation space

`Dict` — one entry per active sensor:

| Sensor key | Shape | dtype |
|------------|-------|-------|
| `CAMERA` / `OBSERVATION` / `LEFT_CAMERA` | `(120, 160, 3)` | uint8 |
| `STEREO` | `(120, 160, 2)` | uint8 |
| `LIDAR` | `(64,)` | float32 |
| `SECTOR_LIDAR` | `(8,)` | float32 |
| `DISCRETIZED_SECTOR_LIDAR` | `(N×M,)` | float32 |

## Reward function parameters

The callable passed as `reward_fn` receives a `dict` with keys from
`deepracer_env.agent_ctrl.constants.RewardParam`, including:

| Key | Type | Description |
|-----|------|-------------|
| `all_wheels_on_track` | bool | All four wheels on track |
| `progress` | float | Lap progress 0–100 % |
| `speed` | float | Current speed m/s |
| `steering_angle` | float | Current steering angle degrees |
| `distance_from_center` | float | Distance from track centre line |
| `is_left_of_center` | bool | Car is left of centre line |
| `waypoints` | list | All track waypoints |
| `closest_waypoints` | list[int] | Indices of previous and next waypoints |

## Customisation

### Sensors

Pass a list of sensor keys from `deepracer_env.sensors.constants.Input`:

```python
from deepracer_env.sensors.constants import Input

env = DeepRacerEnv(
    reward_fn=my_reward,
    sensors=[Input.CAMERA.value, Input.LIDAR.value],
)
```

Available sensors: `CAMERA`, `LEFT_CAMERA`, `STEREO`, `LIDAR`, `SECTOR_LIDAR`,
`DISCRETIZED_SECTOR_LIDAR`.

### Config overrides

```python
import deepracer_env.agent_ctrl.constants as ctrl_const

env = DeepRacerEnv(
    reward_fn=my_reward,
    config={
        ctrl_const.ConfigParams.COLLISION_PENALTY.value: 5.0,
        ctrl_const.ConfigParams.OFF_TRACK_PENALTY.value: 2.0,
    },
)
```

### Full control

Pass a pre-built `Agent` instance to bypass all defaults:

```python
from deepracer_env.agents.agent import Agent

env = DeepRacerEnv(agent=my_custom_agent)
```

## Building the Docker image

```bash
./build.sh
```

Uses Docker BuildKit multi-stage builds (`docker/Dockerfile.build-bundle` and
`docker/Dockerfile.combined`). The `deepracer_env` Python package is copied into
the image alongside the ROS colcon workspace. See `build.sh` for architecture
flags (`-a cpu` / `-a gpu`).

## License

Apache License 2.0 — see [LICENSE](LICENSE).
