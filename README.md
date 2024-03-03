# AWS DeepRacer Robomaker Image
This repository contains the extracts from the AWS DeepRacer Robomaker simapp; including
* the environment and tracks
* the robotics agent code ('markov')
* Dockerfile

## Available pre-built versions

In most cases it will be sufficient to use one of our pre-built images which use optimized Tensorflow binaries. 
* CPU images support training using the CPU for inference. 
* GPI images support training using CUDA GPU for inference.

Both containers support OpenGL acceleration.

Built images are available via `docker pull awsdeepracercommunity/deepracer-robomaker:<tag>`. 

### Version 5.2

Version 5.2 is the current development version for 2024, and it is receiving both new tracks as well as functionality updates as they are released by AWS.

5.2 introduces an upgrade to Ubuntu 20.04, ROS Noetic, Gazebo 11 and Tensorflow 2.

| Version  | Comment         | CPU      | GPU      |
| -------- | -------------- | -------- | -------- | 
| 5.2.0-dev       | Initial release  |  `5.2.0-dev-cpu`, `5.2.0-dev-cpu-avx2` | `5.2.0-dev-gpu` |

### Version 5.1

Version 5.1 was the development version for 2023. It is now deprecated.

| Version  | Comment         | AVX      | AVX2     | GPU      |
| -------- | -------------- | -------- | -------- | -------- | 
| 5.1.0       | Initial release  |  `5.1.0-cpu-avx` | `5.1.0-cpu-avx2` | `5.1.0-gpu` |
| 5.1.1       | CUDA 11.4.4 + misc  |  `5.1.1-cpu-avx` | `5.1.1-cpu-avx2` | `5.1.1-gpu` |
| 5.1.2       | Node management + misc. |  `5.1.2-cpu-avx` | `5.1.2-cpu-avx2` | `5.1.2-gpu` |

## Building the image

A build script is available as `build.sh`. By default both images will be built (`cpu` and `gpu` for version `5.3`). Use the `-a` switch to limit number of images.

### Tensorflow build

As of Version 5.2 the standard PyPI Tensorflow image is being used, which supports CPU and GPU inference.

## Development build

To get a folder compatible with DRfCs `DR_ROBOMAKER_MOUNT_SIMAPP_DIR` use the `build-dev-bundle.sh` script. Through using a build container it will create a `./install` directory that can be mounted in Robomaker. 

