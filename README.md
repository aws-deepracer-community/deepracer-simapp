# AWS DeepRacer Robomaker Image
This repository contains the extracts from the AWS DeepRacer Robomaker simapp; including
* the environment and tracks
* the robotics agent code ('markov')
* Dockerfile

The built image is available via `docker pull awsdeepracercommunity/deepracer-robomaker:cpu`

## Building the image

To build the image run `docker build -t awsdeepracercommunity/deepracer-robomaker:cpu -f docker/Dockerfile.cpu .`

## Available pre-built versions

Available tags are:

| Version  | Status         | AVX      | AVX2     | AVX512   | GPU      |
| -------- | -------------- | -------- | -------- | -------- | -------- | 
| Release  | 2.0.1          | `cpu`, `cpu-avx` | `cpu-avx2` | `cpu-avx512` | `gpu`| 
| 2.0.1    | Update 29-MAR  | `2.0.1-cpu-avx`, `2.0.1-cpu` | `2.0.1-cpu-avx2` | `2.0.1-cpu-avx512` | `2.0.1-gpu` |
| 2.0.2    | Update 01-APR  |          |          | `2.0.2-cpu-avx512` | 

## Building the image

To build the image run `docker build -t awsdeepracercommunity/deepracer-robomaker:cpu -f docker/Dockerfile.cpu .`

### Tensorflow build

Tensorflow has been built with the following parameters:
* `cpu-avx` with `-march=x86-64 -maes -mavx -mmmx -mpclmul -mpopcnt -msse -msse2 -msse3 -msse4.1 -msse4.2 -mssse3`
* `cpu-avx2` with `-march=x86-64 -maes -mavx -mavx2 -mmmx -mpclmul -mpopcnt -msse -msse2 -msse3 -msse4.1 -msse4.2 -mssse3 -mf16c -mfma`
