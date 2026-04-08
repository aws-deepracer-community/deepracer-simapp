#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
    echo "Requested to stop."
    exit 1
}

set -euo pipefail

PREFIX="awsdeepracercommunity"
VARIANTS="cpu gpu"
PLATFORM=""

function usage() {
    cat <<EOF
Usage: $0 [-a "cpu gpu"] [--platform linux/amd64|linux/arm64] [-p prefix] [-f] [-c] [--push]
  -a, --variant       Space-separated variants to build (default: "cpu gpu")
      --platform      Docker platform to build for (default: native host)
  -p, --prefix        Image prefix / repository namespace
  -f, --no-cache      Disable Docker build cache
  -c, --rebuild-core  Force rebuild of the core builder image
      --push          Push built leaf images after the build completes
EOF
}

function detect_platform() {
    case "$(uname -m)" in
    x86_64|amd64)
        echo "linux/amd64"
        ;;
    aarch64|arm64)
        echo "linux/arm64"
        ;;
    *)
        echo "Unsupported host architecture: $(uname -m)" >&2
        exit 1
        ;;
    esac
}

while [[ $# -gt 0 ]]; do
    case "$1" in
    -a|--variant)
        VARIANTS="$2"
        shift 2
        ;;
    --platform)
        PLATFORM="$2"
        shift 2
        ;;
    -p|--prefix)
        PREFIX="$2"
        shift 2
        ;;
    -f|--no-cache)
        OPT_NOCACHE="--no-cache"
        shift
        ;;
    -c|--rebuild-core)
        OPT_CORE="yes"
        shift
        ;;
    --push)
        OPT_PUSH="yes"
        shift
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        echo "Unknown option $1" >&2
        usage
        exit 1
        ;;
    esac
done

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
VERSION=$(jq -r '.simapp' "$DIR/VERSION")
PLATFORM="${PLATFORM:-$(detect_platform)}"

case "$PLATFORM" in
linux/amd64)
    TARGET_ARCH="amd64"
    ;;
linux/arm64)
    TARGET_ARCH="arm64"
    ;;
*)
    echo "Unsupported platform ${PLATFORM}" >&2
    exit 1
    ;;
esac

CORE_TAG="latest-${TARGET_ARCH}"
BUNDLE_TAG="latest-${TARGET_ARCH}"

if [ "$(docker images -q ${PREFIX}/deepracer-simapp-build-core:${CORE_TAG} 2>/dev/null)" == "" ] || [ -n "${OPT_NOCACHE:-}" ] || [ -n "${OPT_CORE:-}" ]; then
    echo "Preparing core builder image ${PREFIX}/deepracer-simapp-build-core:${CORE_TAG} for ${PLATFORM}..."
    docker buildx build ${OPT_NOCACHE:-} --load --platform "${PLATFORM}" \
        -t ${PREFIX}/deepracer-simapp-build-core:${CORE_TAG} \
        -f docker/Dockerfile.build-core .
else
    echo "Core builder image ${PREFIX}/deepracer-simapp-build-core:${CORE_TAG} already exists."
fi

echo "Preparing bundle distribution for ${PLATFORM}..."
docker buildx build ${OPT_NOCACHE:-} --load --platform "${PLATFORM}" \
    -t ${PREFIX}/deepracer-simapp-build-bundle:${BUNDLE_TAG} \
    -f docker/Dockerfile.build-bundle \
    --build-arg CORE_PREFIX=${PREFIX} \
    --build-arg CORE_TAG=${CORE_TAG} .

echo "Preparing docker images for [${VARIANTS}] on ${PLATFORM}"

for variant in $VARIANTS; do
    case $variant in
    gpu)
        if [ "${TARGET_ARCH}" != "amd64" ]; then
            echo "Skipping gpu build on ${PLATFORM}; only linux/amd64 is supported."
            continue
        fi
        CORE_IMG="nvcr.io/nvidia/cuda:12.6.3-cudnn-runtime-ubuntu24.04"
        NVCC_VER="cuda-nvcc-12-6"
        ;;
    cpu)
        CORE_IMG="ubuntu:24.04"
        NVCC_VER=""
        ;;
    *)
        echo "Unknown variant ${variant}" >&2
        exit 1
        ;;
    esac

    IMAGE_TAG="${PREFIX}/deepracer-simapp:${VERSION}-${variant}-${TARGET_ARCH}"

    set -x
    docker buildx build . ${OPT_NOCACHE:-} --load --platform "${PLATFORM}" \
        -t "${IMAGE_TAG}" \
        -f docker/Dockerfile.combined \
        --build-arg IMG_VERSION="${VERSION}" \
        --build-arg BUNDLE_PREFIX="${PREFIX}" \
        --build-arg BUNDLE_TAG="${BUNDLE_TAG}" \
        --build-arg CORE_IMG="${CORE_IMG}" \
        --build-arg NVCC_VER="${NVCC_VER}"
    set +x

    if [ -n "${OPT_PUSH:-}" ]; then
        docker push "${IMAGE_TAG}"
    fi
done
