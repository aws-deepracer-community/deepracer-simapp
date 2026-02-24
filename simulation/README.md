# simulation/

This folder contains the **ROS/Gazebo simulation stack** that must be running before
the `deepracer_env` Python package can interact with the virtual racecar.

`deepracer_env` communicates with this stack exclusively through ROS topics and
services — it never imports anything from here at the Python level.

---

## Origin

Derived from the
[AWS DeepRacer Community simapp](https://github.com/aws-deepracer-community/deepracer-simapp)
(Apache-2.0), originally built to run inside AWS RoboMaker.  The SageMaker RL Coach
and Kinesis Video Stream packages that were part of the original simapp have been
retained here but are no longer wired up on the Python side.

---

## Folder structure

```
simulation/
├── src/                          # colcon workspace — ROS packages built at Docker build time
│   ├── deepracer_simulation_environment/   # Launch files, track loading, reward bridge
│   ├── deepracer_gazebo_system_plugin/     # Gazebo physics plugin (wheel & steering control)
│   ├── deepracer_msgs/                     # Custom ROS message & service definitions
│   ├── deepracer_node_monitor/             # Watchdog: restarts crashed ROS nodes
│   ├── node_monitor/                       # Generic node monitor base library
│   ├── aws-robomaker-simulation-ros-pkgs/  # AWS ROS bridge utilities
│   ├── utils-common/                       # Shared C++ utilities
│   ├── utils-ros1/                         # ROS1-specific utility helpers
│   ├── sagemaker_rl_agent/          # ⚠ UNUSED — legacy SageMaker RL Coach agent
│   └── kinesisvideo-common/         # ⚠ UNUSED — legacy Kinesis Video Stream support
│       ros_kvs_streamer/
│
├── meshes/        # 3D surface meshes for every track variant (used by Gazebo)
├── worlds/        # Gazebo .world files (lighting, physics, track placement)
├── models/        # Gazebo model definitions (car, obstacles, track objects)
├── urdf/          # Robot description — racecar joints, sensors, plugins
├── routes/        # Waypoint / centerline data per track (consumed by track_geom/)
├── track_iconography/  # Track map PNG images (used by the simulation UI)
├── script/        # start_deepracer_node_monitor.py — copied into the Docker image
└── configs/       # Reserved for future runtime configuration files
```

---

## Build & launch

The `src/` directory is a standard **colcon workspace**.  It is compiled inside the
Docker image via:

```bash
source /opt/ros/noetic/setup.bash
colcon build
```

The resulting install space (`/opt/bundle/install`) is the artefact that the runtime
container sources.  See `docker/Dockerfile.build-bundle` for the full build steps.

To launch the simulation locally (inside the container):

```bash
source /opt/ros/noetic/setup.bash
source /opt/bundle/install/setup.bash
roslaunch deepracer_simulation_environment deepracer_rl.launch
```

The `deepracer_env` Python package can then be used from a separate process once the
ROS master and all simulation nodes are up.
