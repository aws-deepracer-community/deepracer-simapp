# AWS DeepRacer Simapp Image

This repository contains the extracts from the AWS DeepRacer Simapp; including

- the environment and tracks
- the robotics agent code ('markov')
- Dockerfiles

The project supports both DeepRacer-for-Cloud (DRfC) and deepracer-on-aws (DRoA) image workflows:

- DRfC images: `deepracer-simapp` (CPU and GPU variants)
- DRoA images: `deepracer-on-aws-simapp` and validator images
	(`deepracer-on-aws-model-validation` and `deepracer-on-aws-reward-function-validation`)

In DeepRacer-for-Cloud this single image is being used for three purposes:

- "Robomaker" container (1 or more) providing robotics simulation using ROS and Gazebo
- "Sagemaker" container providing the model training job
- "RL Coach" container which bootstraps the Sagemaker container, using the Sagemaker SDK and Sagemaker Local.

## Source

The code in `bundle/` is primarily provided by AWS, and is closely matching the code that is running in the AWS DeepRacer console. Work is based on the information
provided in [Guidance for training an AWS DeepRacer model using Amazon Sagemaker](https://github.com/aws-solutions-library-samples/guidance-for-training-an-aws-deepracer-model-using-amazon-sagemaker).

The source is taken from the [deepracer-on-aws-simapp Container](https://gallery.ecr.aws/aws-solutions/deepracer-on-aws-simapp), and is replicated here in the `upstream` branch.

## Single image

Up until Version 5.3 these docker containers were run backed by three different containers. As the actual prerequisites (Ubuntu and Python packages) converged
over time, the images were content-wise very similar, but the same code was downloaded more than once increasing network and storage needs.

Previous versions had the Markov code `bundle/markov` injected into Sagemaker containers via the RL coach container (where it was built in); now the codebase
is built into the single image, and RL coach does not inject code anymore.

## Core Technologies / Technology Stack

Installed core technologies are:
 - Ubuntu 24.04
 - Python 3.12
 - Tensorflow 2.20
 - CUDA 12.6 (GPU only)
 - Redis 8.6.1
 - ROS 2 Jazzy
 - Gazebo Harmonic

## Available pre-built versions

In most cases it is sufficient to use one of the published images.

### DRfC images on Docker Hub

- Repository: `awsdeepracercommunity/deepracer-simapp`
- CPU image supports training using CPU inference. Tag: `VERSION-cpu`
- GPU image supports training using CUDA GPU inference. Tag: `VERSION-gpu`

Both containers support OpenGL acceleration. Starting with version 6.0.3 the `-cpu` tag is a transparent wrapper around both `arm64` and `amd64` versions, and Docker selects the matching architecture automatically.

- Docker Hub page: [awsdeepracercommunity/deepracer-simapp](https://hub.docker.com/repository/docker/awsdeepracercommunity/deepracer-simapp)
- Pull example: `docker pull awsdeepracercommunity/deepracer-simapp:<tag>`

### DRoA images on Public ECR

DRoA images are published in the public gallery namespace:
[gallery.ecr.aws/deepracer-community](https://gallery.ecr.aws/deepracer-community)

- `public.ecr.aws/deepracer-community/deepracer-on-aws-simapp:<tag>`
- `public.ecr.aws/deepracer-community/deepracer-on-aws-model-validation:<tag>`
- `public.ecr.aws/deepracer-community/deepracer-on-aws-reward-function-validation:<tag>`

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
