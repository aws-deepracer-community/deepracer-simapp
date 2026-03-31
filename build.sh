#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
    echo "Requested to stop."
    exit 1
}

set -e

PREFIX="awsdeepracercommunity"
ARCH="cpu gpu"

while getopts ":a:fcp:" opt; do
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
    c)
        OPT_CORE="yes"
        ;;
    \?)
        echo "Invalid option -$OPTARG" >&2
        exit 1
        ;;
    esac
done

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
VERSION=$(jq -r '.simapp' $DIR/VERSION)

if [ "$(docker images -q ${PREFIX}/deepracer-simapp-build-core:latest 2> /dev/null)" == "" ] || [ -n "${OPT_NOCACHE}" ] || [ -n "${OPT_CORE}" ]; then
    echo "Preparing core builder image ${PREFIX}/deepracer-simapp-build-core:latest..."
    docker buildx build ${OPT_NOCACHE} -t ${PREFIX}/deepracer-simapp-build-core:latest -f docker/Dockerfile.build-core .
else
    echo "Core builder image ${PREFIX}/deepracer-simapp-build-core:latest already exists."
fi

echo "Preparing bundle distribution..."
docker buildx build ${OPT_NOCACHE} -t ${PREFIX}/deepracer-simapp-build-bundle:latest -f docker/Dockerfile.build-bundle --build-arg CORE_PREFIX=${PREFIX} .

echo "Preparing docker images for [$ARCH]"

for a in $ARCH; do
    case $a in
    gpu)
        CORE_IMG="nvcr.io/nvidia/cuda:12.6.3-cudnn-runtime-ubuntu24.04"
        NVCC_VER="cuda-nvcc-12-6"
        ;;
    cpu)
        CORE_IMG="ubuntu:24.04"
        NVCC_VER=""
        ;;
    esac

    set -x
    docker buildx build . ${OPT_NOCACHE} -t $PREFIX/deepracer-simapp:${VERSION}-${a} -f docker/Dockerfile.combined \
        --build-arg IMG_VERSION=${VERSION} \
        --build-arg BUNDLE_PREFIX=${PREFIX} \
        --build-arg CORE_IMG=${CORE_IMG} \
        --build-arg NVCC_VER=${NVCC_VER}
    set +x
done
