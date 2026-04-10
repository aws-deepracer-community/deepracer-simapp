#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
    echo "Requested to stop."
    exit 1
}

set -euo pipefail

REGISTRY="public.ecr.aws"
PREFIX="deepracer-community"
SIMAPP_PREFIX="${SIMAPP_PREFIX:-awsdeepracercommunity}"
SIMAPP_ARCH="${SIMAPP_ARCH:-amd64}"
BUILD_SIMAPP="yes"
BUILD_VALIDATORS="yes"

function usage() {
    cat <<EOF
Usage: $0 [-p prefix] [-r registry] [-s simapp-prefix] [-f] [--push] [--simapp-only|--validators-only]
  -p, --prefix           Repository prefix / namespace under the target registry
  -r, --registry         Target registry (for example 123456789012.dkr.ecr.eu-central-1.amazonaws.com)
  -s, --simapp-prefix    Source repository prefix for the existing simapp cpu-amd64 image
  -f, --no-cache         Disable Docker build cache for validator builds
      --push             Push the resulting images to the target registry
      --simapp-only      Only tag/push the DROA simapp image
      --validators-only  Only build/push the validator images
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
    -p|--prefix)
        PREFIX="$2"
        shift 2
        ;;
    -r|--registry)
        REGISTRY="$2"
        shift 2
        ;;
    -s|--simapp-prefix)
        SIMAPP_PREFIX="$2"
        shift 2
        ;;
    -f|--no-cache)
        OPT_NOCACHE="--no-cache"
        shift
        ;;
    --push)
        OPT_PUSH="yes"
        shift
        ;;
    --simapp-only)
        BUILD_VALIDATORS=""
        shift
        ;;
    --validators-only)
        BUILD_SIMAPP=""
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
DROA_VERSION=$(jq -r '."deepracer-on-aws"' "$DIR/VERSION")
SIMAPP_VERSION=$(jq -r '.simapp' "$DIR/VERSION")

if [ -z "$REGISTRY" ] || [ -z "$PREFIX" ]; then
    REPO="${REGISTRY}${PREFIX}"
else
    REPO="${REGISTRY}/${PREFIX}"
fi

echo "Building deepracer-on-aws images version ${DROA_VERSION} under ${REPO}"

if [ -n "${BUILD_SIMAPP}" ]; then
    SIMAPP_BUILD_IMG="${SIMAPP_PREFIX}/deepracer-simapp:${SIMAPP_VERSION}-cpu-${SIMAPP_ARCH}"
    SIMAPP_IMG="${REPO}/deepracer-on-aws-simapp:${DROA_VERSION}"

    if [ "$(docker images -q ${SIMAPP_BUILD_IMG} 2>/dev/null)" == "" ] || [ -n "${OPT_NOCACHE:-}" ]; then
        echo "SimApp image ${SIMAPP_BUILD_IMG} not found locally - running build.sh..."
        "$DIR/build.sh" -a cpu --platform "linux/${SIMAPP_ARCH}" -p "${SIMAPP_PREFIX}" ${OPT_NOCACHE:+-f}
    else
        echo "SimApp image ${SIMAPP_BUILD_IMG} already exists, reusing it for the DROA tag."
    fi

    docker tag "${SIMAPP_BUILD_IMG}" "${SIMAPP_IMG}"

    if [ -n "${OPT_PUSH:-}" ]; then
        docker push "${SIMAPP_IMG}"
    fi
fi

if [ -n "${BUILD_VALIDATORS}" ]; then
    VALIDATOR_OUTPUT="--load"
    if [ -n "${OPT_PUSH:-}" ]; then
        VALIDATOR_OUTPUT="--push"
    fi

    echo "Building model-validation image..."
    docker buildx build . ${OPT_NOCACHE:-} ${VALIDATOR_OUTPUT} --platform linux/amd64 \
        -t ${REPO}/deepracer-on-aws-model-validation:${DROA_VERSION} \
        -f docker/Dockerfile.model-validation \
        --build-arg DROA_VERSION=${DROA_VERSION}

    echo "Building reward-function-validation image..."
    docker buildx build . ${OPT_NOCACHE:-} ${VALIDATOR_OUTPUT} --platform linux/amd64 \
        -t ${REPO}/deepracer-on-aws-reward-function-validation:${DROA_VERSION} \
        -f docker/Dockerfile.reward-function-validation
fi

echo "Done. Built images:"
if [ -n "${BUILD_SIMAPP}" ]; then
    echo "  ${REPO}/deepracer-on-aws-simapp:${DROA_VERSION}"
fi
if [ -n "${BUILD_VALIDATORS}" ]; then
    echo "  ${REPO}/deepracer-on-aws-model-validation:${DROA_VERSION}"
    echo "  ${REPO}/deepracer-on-aws-reward-function-validation:${DROA_VERSION}"
fi
