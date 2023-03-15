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

Built images are available via `docker pull awsdeepracercommunity/deepracer-robomaker:<tag>`. 

Available tags are:

### Version 5

Version 5 is the current development version for 2022, and it is receiving both new tracks as well as functionality updates as they are released by AWS.

| Version  | Comment         | AVX      | AVX2     | GPU      |
| -------- | -------------- | -------- | -------- | -------- | 
| 5.0.0       | Initial release  |  `5.0.0-cpu-avx` `5.0.0-cpu-gl-avx`  | `5.0.0-cpu-avx2` `5.0.0-cpu-gl-avx2` | `5.0.0-gpu` `5.0.0-gpu-gl` |
| 5.0.1       | April Track  |  `5.0.1-cpu-avx` `5.0.1-cpu-gl-avx`  | `5.0.1-cpu-avx2` `5.0.1-cpu-gl-avx2` | `5.0.1-gpu` `5.0.1-gpu-gl` |
| 5.0.2       | May Track  |  `5.0.2-cpu-avx` `5.0.2-cpu-gl-avx`  | `5.0.2-cpu-avx2` `5.0.2-cpu-gl-avx2` | `5.0.2-gpu` `5.0.2-gpu-gl` |
| 5.0.3      | June Track  |  `5.0.3-cpu-avx` `5.0.3-cpu-gl-avx`  | `5.0.3-cpu-avx2` `5.0.3-cpu-gl-avx2` | `5.0.3-gpu` `5.0.3-gpu-gl` |
| 5.0.4      | July Track  |  `5.0.4-cpu-avx` `5.0.4-cpu-gl-avx`  | `5.0.4-cpu-avx2` `5.0.4-cpu-gl-avx2` | `5.0.4-gpu` `5.0.4-gpu-gl` |
| 5.0.5      | August Track  |  `5.0.5-cpu-avx` `5.0.5-cpu-gl-avx`  | `5.0.5-cpu-avx2` `5.0.5-cpu-gl-avx2` | `5.0.5-gpu` `5.0.5-gpu-gl` |
| 5.0.6      | September Track  |  `5.0.6-cpu-avx` `5.0.6-cpu-gl-avx`  | `5.0.6-cpu-avx2` `5.0.6-cpu-gl-avx2` | `5.0.6-gpu` `5.0.6-gpu-gl` |
| 5.0.7      | October & Reinvent 2022 Tracks  |  `5.0.7-cpu-avx` `5.0.7-cpu-gl-avx`  | `5.0.7-cpu-avx2` `5.0.7-cpu-gl-avx2` | `5.0.7-gpu` `5.0.7-gpu-gl` |
| 5.0.8      | Updates to Reinvent 2022 Tracks  |  `5.0.8-cpu-avx` `5.0.8-cpu-gl-avx`  | `5.0.8-cpu-avx2` `5.0.8-cpu-gl-avx2` | `5.0.8-gpu` `5.0.8-gpu-gl` |
| 5.0.9      | Minor updates and fixes  |  `5.0.9-cpu-avx` `5.0.9-cpu-gl-avx`  | `5.0.9-cpu-avx2` `5.0.9-cpu-gl-avx2` | `5.0.9-gpu` `5.0.9-gpu-gl` |
| 5.0.10 (*)     | Tracks expanded with CW and CCW tracks  |  `5.0.10-cpu-avx` `5.0.10-cpu-gl-avx`  | `5.0.10-cpu-avx2` `5.0.10-cpu-gl-avx2` | `5.0.10-gpu` `5.0.10-gpu-gl` |

(*) 5.0.10 received a patch on 15-MAR-2023 due to a correction of the `arctic_pro_ccw` and `arctic_pro_cw` tracks.

### Version 4

Version 4 is retired.

| Version  | Comment         | AVX      | AVX2     | GPU      |
| -------- | -------------- | -------- | -------- | -------- | 
| 4.0.13       | 2022 March Track  |  `4.0.13-cpu-avx` `4.0.13-cpu-gl-avx`  | `4.0.13-cpu-avx2` `4.0.13-cpu-gl-avx` | `4.0.13-gpu` `4.0.13-gpu-gl` |

## Building the image

To build the CPU image run `docker build -t awsdeepracercommunity/deepracer-robomaker:cpu -f docker/Dockerfile.cpu . --build-arg TENSORFLOW_VER='tensorflow==1.11.0`

To build the GPU image run `docker build -t awsdeepracercommunity/deepracer-robomaker:gpu -f docker/Dockerfile.gpu . `

To add a custom tensorflow library run `docker build -t awsdeepracercommunity/deepracer-robomaker:cpu -f docker/Dockerfile.cpu . --build-arg TENSORFLOW_VER=<http-url-to-file>`

### Tensorflow build

Tensorflow has been built with the following parameters:
* `cpu-avx` with `-march=x86-64 -maes -mavx -mmmx -mpclmul -mpopcnt -msse -msse2 -msse3 -msse4.1 -msse4.2 -mssse3`
* `cpu-avx2` with `-march=x86-64 -maes -mavx -mavx2 -mmmx -mpclmul -mpopcnt -msse -msse2 -msse3 -msse4.1 -msse4.2 -mssse3 -mf16c -mfma`
