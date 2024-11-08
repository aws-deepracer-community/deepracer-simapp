# AWS DeepRacer Simapp Image

This repository contains the extracts from the AWS DeepRacer Simapp; including

- the environment and tracks
- the robotics agent code ('markov')
- Dockerfiles

In DeepRacer-for-Cloud this single image is being used for three purposes:

- "Robomaker" container (1 or more) providing robotics simulation using ROS and Gazebo
- "Sagemaker" container providing the model training job
- "RL Coach" container which bootstraps the Sagemaker container, using the Sagemaker SDK and Sagemaker Local.

## Source

The code in `bundle/` is primarily provided by AWS, and is closely matching the code that is running in the AWS DeepRacer console. Work is based on the information
provided in [Guidance for training an AWS DeepRacer model using Amazon Sagemaker](https://github.com/aws-solutions-library-samples/guidance-for-training-an-aws-deepracer-model-using-amazon-sagemaker).

The source is taken from the [deepracer-sim-public Container](https://gallery.ecr.aws/k1d3r4z1/deepracer-sim-public), and is replicated here in the `upstream` branch.

## Single image

Up until Version 5.3 these docker containers were run backed by three different containers. As the actual prerequisites (Ubuntu and Python packages) converged
over time, the images were content-wise very similar, but the same code was downloaded more than once increasing network and storage needs.

Previous versions had the Markov code `bundle/markov` injected into Sagemaker containers via the RL coach container (where it was built in); now the codebase
is built into the single image, and RL coach does not inject code anymore.

## Core Technologies / Technology Stack

Installed core technologies are:
 - Ubuntu 20.04
 - Python 3.8
 - Tensorflow 2.13
 - CUDA 11.8 (GPU only)
 - REDIS 6.2.7
 - ROS Noetic
 - Gazebo 11

## Available pre-built versions

In most cases it will be sufficient to use one of our pre-built images:

- CPU image support training using the CPU for inference. (Tag `VERSION-cpu`)
- GPU image support training using CUDA GPU for inference. (Tag `VERSION-gpu`)

Both containers support OpenGL acceleration.

Built images are available via `docker pull awsdeepracercommunity/deepracer-simapp:<tag>` - see also [Docker Hub](https://hub.docker.com/repository/docker/awsdeepracercommunity/deepracer-simapp).

## Building the image

A build script is available as `build.sh`. By default both images will be built (`cpu` and `gpu` for version `5.3`). Use the `-a` switch to limit number of images.

- Clone this repo into the home directory of your Linux machine
- run `./utils/extract-and-diff.sh` (which pulls changes from the AWS provided image into upstream branch)
- Commit the changes to the upstream branch
- Change to the branch you want to build from (e.g. dev or a branch created from dev)
- Cherry pick changes from the commit in the upstream branch into the branch you want to build from (e.g. dev or a branch created from dev)
- Login to dockerhub (permissions to push to the awsdeepracercommunity community  required)
- update VERSION file in root of simapp repo to the correct version tag
- run `./build.sh -f`
- push images (e.g. `docker image push awsdeepracercommunity/deepracer-simapp:X.Y.Z-cpu` and `docker image push awsdeepracercommunity/deepracer-simapp:X.Y.Z-gpu`)

## Development build

To get a folder compatible with DRfCs `DR_ROBOMAKER_MOUNT_SIMAPP_DIR` use the `build-dev-bundle.sh` script. Through using a build container it will create a `./install` directory that can be mounted in Robomaker.
