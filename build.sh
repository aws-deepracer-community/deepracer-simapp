#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
    echo "Requested to stop."
    exit 1
}

set -e

PREFIX="awsdeepracercommunity"
ARCH="cpu gpu"

while getopts ":a:fp:" opt; do
    case $opt in
    a)
        ARCH="$OPTARG"
        ;;
    p)
        PREFIX="$OPTARG"
        ;;
    f)
        OPT_NOCACHE="--no-cache"
        ;;
    \?)
        echo "Invalid option -$OPTARG" >&2
        exit 1
        ;;
    esac
done

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
VERSION=$(cat $DIR/VERSION)

if [ "$(docker images -q ${PREFIX}/deepracer-robomaker-build-core:latest 2> /dev/null)" == "" ] || [ -n "${OPT_NOCACHE}" ]; then
    echo "Preparing core builder image ${PREFIX}/deepracer-robomaker-build-core:latest..."
    docker buildx build ${OPT_NOCACHE} -t ${PREFIX}/deepracer-robomaker-build-core:latest -f docker/Dockerfile.build-core .
else
    echo "Core builder image ${PREFIX}/deepracer-robomaker-build-core:latest already exists."
fi

echo "Preparing bundle distribution..."
docker buildx build ${OPT_NOCACHE} -t ${PREFIX}/deepracer-robomaker-build-bundle:latest -f docker/Dockerfile.build-bundle --build-arg BUILDER_PREFIX=${PREFIX} .

echo "Preparing docker images for [$ARCH]"

for a in $ARCH; do
    case $a in
    gpu)
        CORE_IMG="nvidia/cuda:11.8.0-cudnn8-runtime-ubuntu20.04"
        NVCC_VER="cuda-nvcc-11-8"
        TF_VER="tensorflow==2.13.1 tensorflow-probability==0.21.0"
        ;;
    cpu)
        CORE_IMG="ubuntu:20.04"
        NVCC_VER=""
        TF_VER="tensorflow-cpu==2.13.1"
        ;;
    esac

    set -x
    docker buildx build . ${OPT_NOCACHE} -t $PREFIX/deepracer-simapp:${VERSION}-${a} -f docker/Dockerfile.combined \
        --build-arg IMG_VERSION=${VERSION} \
        --build-arg BUNDLE_PREFIX=${PREFIX} \
        --build-arg CORE_IMG=${CORE_IMG} \
        --build-arg NVCC_VER=${NVCC_VER} \
        --build-arg TF_VER=${TF_VER}
    set +x
done
