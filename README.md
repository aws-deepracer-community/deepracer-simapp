# deepracer-env

A [Gymnasium](https://gymnasium.farama.org/) environment for the AWS DeepRacer
autonomous-driving simulation, powered by **ROS Noetic** and **Gazebo 11**.

Instead of training against a fixed network with a plug-in reward function, this
repository exposes the full simulation as a standard `gymnasium.Env` so you can
bring any RL algorithm or neural-network architecture you like.

---

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
├── simulation/             # ROS/Gazebo workspace (meshes, worlds, URDF, routes)
│   ├── src/                # ROS colcon workspace — built inside Docker
│   ├── meshes/
│   ├── worlds/
│   └── urdf/
├── docker/                 # Dockerfiles and pip requirements for the container
│   ├── Dockerfile.base     # OS + ROS Noetic + Gazebo 11 system dependencies
│   ├── Dockerfile.build    # Compiles simulation/src/ with colcon
│   ├── Dockerfile.runtime  # Final runnable image
│   └── Dockerfile.dev-user # Dev variant: adds a non-root local user
├── examples/
│   └── train.py            # Minimal PPO training example (Stable-Baselines3)
├── sample-config/          # Example reward function and model metadata
├── pyproject.toml          # Package metadata and build configuration
├── requirements.txt        # Minimal Python runtime dependencies
├── build.sh                # Builds the Docker image chain
├── build-dev-bundle.sh     # Developer workflow: colcon build into local volume
└── VERSION
```

---

## How this fits into your project

**`deepracer_env` communicates with Gazebo exclusively through ROS**, which means
it can only run inside an environment that has ROS Noetic installed.  It cannot
be used as a standalone Python package on a bare machine.

The intended usage pattern is:

```
your-project/               ← your RL project repository
├── reward.py
├── train.py
├── pyproject.toml          ← depends on deepracer-env
└── Dockerfile              ← FROM the simulation image built here
```

Your training code and `deepracer_env` both run **inside a container that extends
the simulation image produced by this repository**.  You edit your code locally;
the container provides the ROS + Gazebo infrastructure.

---

## Step-by-step: using deepracer-env in your project

### Step 1 — Build the simulation image

Clone this repository and build the runtime Docker image:

```bash
git clone https://github.com/your-org/deepracer-env.git
cd deepracer-env
./build.sh -a cpu          # or -a gpu  for NVIDIA GPU support
```

This produces the image `awsdeepracercommunity/deepracer-env:<VERSION>-cpu`.

### Step 2 — Create your project repository

Create a new repository with this minimal structure:

```
my-deepracer-project/
├── reward.py
├── train.py
├── requirements.txt
├── pyproject.toml
└── Dockerfile
```

#### `pyproject.toml`

Do **not** list `deepracer-env` as a dependency here — it is already installed
in the base simulation image. Only declare your own project's dependencies:

```toml
[project]
name = "my-deepracer-project"
version = "0.1.0"
requires-python = ">=3.8"
dependencies = [
    "stable-baselines3",
]
```

#### `requirements.txt`

List the Python packages your training script directly imports e.g.:

```
stable-baselines3
```

Add any other packages your `train.py` requires (e.g. `numpy`, `pandas`, `wandb`).

#### `Dockerfile`

Extend the simulation image, install your dependencies first (for better layer
caching), then install your project on top of it:

```dockerfile
ARG SIMAPP_TAG=latest-cpu
FROM awsdeepracercommunity/deepracer-env:${SIMAPP_TAG}

# Install third-party dependencies first — cached unless requirements.txt changes
COPY requirements.txt /workspace/requirements.txt
RUN pip install --no-cache-dir -r /workspace/requirements.txt

# Copy and install your project
COPY . /workspace
RUN pip install --no-cache-dir -e /workspace

# Default: run your training script
ENTRYPOINT ["/bin/bash", "-c"]
CMD ["source /opt/ros/noetic/setup.bash && source /opt/simapp/setup.bash && ./run.sh run deepracer_env.launch & sleep 5 && python3 /workspace/train.py"]
```

#### `train.py`

```python
from deepracer_env.environments.deepracer_env import DeepRacerEnv
from stable_baselines3 import PPO


def reward_function(params: dict) -> float:
    if not params["all_wheels_on_track"]:
        return 1e-3
    return float(params["progress"] * params["speed"] / 4.0)


env = DeepRacerEnv(reward_fn=reward_function)

model = PPO("MultiInputPolicy", env, verbose=1)
model.learn(total_timesteps=100_000)
model.save("my_model")
env.close()
```

### Step 3 — Build and run your project image

Inside your project repository:

```bash
docker build \
  --build-arg SIMAPP_TAG=<VERSION>-cpu \
  -t my-deepracer-project:latest .

docker run --rm \
  -e WORLD_NAME=reinvent_base \
  -e ENABLE_GUI=False \
  my-deepracer-project:latest
```

Set `ENABLE_GUI=True` and expose port `5900` to view the Gazebo simulation
over VNC:

```bash
docker run --rm \
  -e WORLD_NAME=reinvent_base \
  -e ENABLE_GUI=True \
  -p 5900:5900 \
  my-deepracer-project:latest
```

Then connect any VNC viewer to `localhost:5900`.

### Step 4 — Iterate without rebuilding the image

Mount your project source as a volume to pick up code changes instantly without
rebuilding:

```bash
docker run --rm \
  -e WORLD_NAME=reinvent_base \
  -v $(pwd):/workspace \
  my-deepracer-project:latest \
  bash -c "source /opt/ros/noetic/setup.bash && source /opt/simapp/setup.bash && ./run.sh run deepracer_env.launch & sleep 5 && python3 /workspace/train.py"
```

---

## API reference

### Action space

`Box([-30, 0.1], [30, 4.0], dtype=float32)` — `[steering_angle_deg, speed_m_s]`

### Observation space

`Dict` — one entry per active sensor:

| Sensor key | Shape | dtype |
|------------|-------|-------|
| `CAMERA` / `OBSERVATION` / `LEFT_CAMERA` | `(120, 160, 3)` | uint8 |
| `STEREO` | `(120, 160, 2)` | uint8 |
| `LIDAR` | `(64,)` | float32 |
| `SECTOR_LIDAR` | `(8,)` | float32 |
| `DISCRETIZED_SECTOR_LIDAR` | `(N×M,)` | float32 |

### Reward function parameters

The callable passed as `reward_fn` receives a `dict` with the following keys:

| Key | Type | Description |
|-----|------|-------------|
| `all_wheels_on_track` | bool | All four wheels on track |
| `progress` | float | Lap progress 0–100 % |
| `speed` | float | Current speed m/s |
| `steering_angle` | float | Current steering angle in degrees |
| `distance_from_center` | float | Distance from track centre line |
| `is_left_of_center` | bool | Car is left of centre line |
| `waypoints` | list | All track waypoints |
| `closest_waypoints` | list[int] | Indices of previous and next waypoints |

### Customising sensors

```python
from deepracer_env.sensors.constants import Input

env = DeepRacerEnv(
    reward_fn=my_reward,
    sensors=[Input.CAMERA.value, Input.LIDAR.value],
)
```

Available sensors: `CAMERA`, `LEFT_CAMERA`, `STEREO`, `LIDAR`, `SECTOR_LIDAR`,
`DISCRETIZED_SECTOR_LIDAR`.

### Customising controller parameters

```python
import deepracer_env.agent_ctrl.constants as ctrl_const

env = DeepRacerEnv(
    reward_fn=my_reward,
    config={
        ctrl_const.ConfigParams.COLLISION_PENALTY.value: 5.0,
        ctrl_const.ConfigParams.OFF_TRACK_PENALTY.value: 2.0,
        ctrl_const.ConfigParams.NUMBER_OF_RESETS.value: 0,
    },
)
```

### Full control — bring your own Agent

Build an `Agent` yourself and pass it directly; `reward_fn`, `sensors`, and
`config` are then ignored:

```python
from deepracer_env.agents.agent import Agent
from deepracer_env.sensors.composite_sensor import CompositeSensor
from deepracer_env.agent_ctrl.rollout_agent_ctrl import RolloutCtrl

sensor = CompositeSensor()
ctrl   = RolloutCtrl(my_config, my_metrics, is_training=True)
env    = DeepRacerEnv(agent=Agent(sensor, ctrl))
```

---

## Building the simulation image

```bash
./build.sh -a cpu    # CPU-only image
./build.sh -a gpu    # NVIDIA GPU image (requires nvidia-docker)
./build.sh -a "cpu gpu" -p myreeg  # both architectures, custom registry prefix
```

The build chain:

1. `Dockerfile.base` — installs ROS Noetic + Gazebo 11 into Ubuntu 20.04
2. `Dockerfile.build` — compiles `simulation/src/` with colcon
3. `Dockerfile.runtime` — assembles the final image

## Developer workflow

To compile the ROS packages locally (output on your disk rather than baked into
an image):

```bash
./build-dev-bundle.sh          # compiles into ./install/
./build-dev-bundle.sh -g       # also starts Gazebo (requires DR_SIMAPP_IMAGE and DR_WORLD_NAME)
```

---

## License

Apache License 2.0 — see [LICENSE](LICENSE).
