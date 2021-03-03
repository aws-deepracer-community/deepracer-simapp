# AWS DeepRacer Robomaker Image
This repository contains the extracts from the AWS DeepRacer Robomaker simapp; including
* the environment and tracks
* the robotics agent code ('markov')
* Dockerfile

## Available pre-built versions

In most cases it will be sufficient to use one of our pre-built images which use optimized Tensorflow binaries. Depending on how modern processor you have available you can pick which instruction sets to use.
* AVX is available on most Intel Core processors from the second generation on - running DeepRacer on an older processor really does not make sense; hence we do not have pre-built images without this instruction set.
* AVX2 is available on more modern Intel Core processors; starting from the Haswell architecture (4th generation), and is probably the image most will use.
* AVX-512 was available on newer Xeon and Core i7-X processors. Due to convergence issues it has been removed.
* GPU uses Tensorflow-GPU instead of CPU. It is not compiled with special instruction sets for the CPU.

Available tags are:

### Version 3

Version 3 is the current version, and it is receiving both new tracks as well as functionality updates as they are released by AWS.

| Version  | Comment         | AVX      | AVX2     | GPU      |
| -------- | -------------- | -------- | -------- | -------- | 
| 3.0.1   | Upgrade to Simapp 3  |  `3.0.1-cpu-avx` `3.0.1-cpu-gl-avx`  | `3.0.1-cpu-avx2` `3.0.1-cpu-gl-avx` | `3.0.1-gpu` `3.0.1-gpu-gl` |
| 3.0.2   | September Track  |  `3.0.2-cpu-avx` `3.0.2-cpu-gl-avx`  | `3.0.2-cpu-avx2` `3.0.2-cpu-gl-avx` | `3.0.2-gpu` `3.0.2-gpu-gl` |
| 3.0.3   | October Track  |  `3.0.3-cpu-avx` `3.0.3-cpu-gl-avx`  | `3.0.3-cpu-avx2` `3.0.3-cpu-gl-avx` | `3.0.3-gpu` `3.0.3-gpu-gl` |
| 3.0.4   | Code fixes  |  `3.0.4-cpu-avx` `3.0.4-cpu-gl-avx`  | `3.0.4-cpu-avx2` `3.0.4-cpu-gl-avx` | `3.0.4-gpu` `3.0.4-gpu-gl` |
| 3.0.5   | Reinvent 2020 Tracks  |  `3.0.5-cpu-avx` `3.0.5-cpu-gl-avx`  | `3.0.5-cpu-avx2` `3.0.5-cpu-gl-avx` | `3.0.5-gpu` `3.0.5-gpu-gl` |
| 3.0.6   | Updated Reinvent 2020 Tracks  |  `3.0.6-cpu-avx` `3.0.6-cpu-gl-avx`  | `3.0.6-cpu-avx2` `3.0.6-cpu-gl-avx` | `3.0.6-gpu` `3.0.6-gpu-gl` |
| 3.1.0   | December features  |  `3.1.0-cpu-avx` `3.1.0-cpu-gl-avx`  | `3.1.0-cpu-avx2` `3.1.0-cpu-gl-avx` | `3.1.0-gpu` `3.1.0-gpu-gl` |
| 3.1.1   | January features  |  `3.1.1-cpu-avx` `3.1.1-cpu-gl-avx`  | `3.1.1-cpu-avx2` `3.1.1-cpu-gl-avx` | `3.1.1-gpu` `3.1.1-gpu-gl` |
| 3.1.2   | March Track  |  `3.1.2-cpu-avx` `3.1.2-cpu-gl-avx`  | `3.1.2-cpu-avx2` `3.1.2-cpu-gl-avx` | `3.1.2-gpu` `3.1.2-gpu-gl` |

### Version 2

Version 2 is in retired state, and will no longer get updates.

| Version  | Comment         | AVX      | AVX2     | GPU      |
| -------- | -------------- | -------- | -------- | -------- | 
| Release  | Version 2.0.11  | `cpu`, `cpu-avx`, `cpu-gl-avx` | `cpu-avx2`, `cpu-gl-avx2`  | `gpu`, `gpu-gl` | 
| 2.0.14   | Reinvent 2020 Tracks   |  `2.0.14-cpu-avx` `2.0.14-cpu-gl-avx`  | `2.0.14-cpu-avx2` `2.0.14-cpu-gl-avx` | `2.0.14-gpu` `2.0.14-gpu-gl` |

The built image is available via `docker pull awsdeepracercommunity/deepracer-robomaker:cpu`. For other images replace `cpu` with the tag of your choice from the above table.

## Building the image

To build the CPU image run `docker build -t awsdeepracercommunity/deepracer-robomaker:cpu -f docker/Dockerfile.cpu . --build-arg TENSORFLOW_VER='tensorflow==1.11.0`

To build the GPU image run `docker build -t awsdeepracercommunity/deepracer-robomaker:gpu -f docker/Dockerfile.gpu . `

To add a custom tensorflow library run `docker build -t awsdeepracercommunity/deepracer-robomaker:cpu -f docker/Dockerfile.cpu . --build-arg TENSORFLOW_VER=<http-url-to-file>`

### Tensorflow build

Tensorflow has been built with the following parameters:
* `cpu-avx` with `-march=x86-64 -maes -mavx -mmmx -mpclmul -mpopcnt -msse -msse2 -msse3 -msse4.1 -msse4.2 -mssse3`
* `cpu-avx2` with `-march=x86-64 -maes -mavx -mavx2 -mmmx -mpclmul -mpopcnt -msse -msse2 -msse3 -msse4.1 -msse4.2 -mssse3 -mf16c -mfma`
