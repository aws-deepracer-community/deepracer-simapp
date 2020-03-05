# AWS DeepRacer Robomaker Image
This repository contains the extracts from the AWS DeepRacer Robomaker simapp; including
* the environment and tracks
* the robotics agent code ('markov')
* Dockerfile

The built image is available via `docker pull awsdeepracercommunity/deepracer-robomaker:cpu`

## Building the image

To build the image run `docker build -t awsdeepracercommunity/deepracer-robomaker:cpu -f docker/Dockerfile.cpu .`
