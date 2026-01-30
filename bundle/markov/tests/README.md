# Tests for AwsSilverstoneMarkovScripts

This directory contains tests for validating the migration from ROS1 to ROS2 and functional tests for the package components.

## Prerequisites

- Docker container with the following dependencies installed:
  - Ubuntu 24.04
  - ROS2 Jazzy
  - Gazebo Harmonic
  - Python 3.10+

## Entering the Docker Container

Before running the tests, you need to enter the Docker container:

```bash
# Enter the Docker container
docker exec -it <container-name> bash
```

## Tests

### 1. ROS2 Migration Tests

These tests validate the basic ROS2 functionality:

```bash
# Source the ROS2 environment setup file
source /opt/ros/jazzy/setup.bash

# Run the ROS2 migration tests
cd /path/to/AwsSilverstoneMarkovScripts
python3 -m markov.tests.test_ros2_migration -v
```

**Test Coverage:**
- ROS2 node initialization and management
- Service client functionality
- Updated Gazebo Harmonic service names
- New SimApp version constants