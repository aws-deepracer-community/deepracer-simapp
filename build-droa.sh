#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
    echo "Requested to stop."
    exit 1
}

set -e

REGISTRY="public.ecr.aws"
PREFIX="deepracer-community"
SIMAPP_PREFIX="awsdeepracercommunity"

while getopts ":fp:r:" opt; do
    case $opt in
    p)
        PREFIX="$OPTARG"
        ;;
    r)
        REGISTRY="$OPTARG"
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
DROA_VERSION=$(jq -r '."deepracer-on-aws"' "$DIR/VERSION")
SIMAPP_VERSION=$(jq -r '.simapp' "$DIR/VERSION")

if [ -z "$REGISTRY" ] || [ -z "$PREFIX" ]; then
    REPO="${REGISTRY}${PREFIX}"
else
    REPO="${REGISTRY}/${PREFIX}"
fi

echo "Building deepracer-on-aws images version ${DROA_VERSION} under ${REPO}"

# --- SimApp (cpu) ---
SIMAPP_BUILD_IMG="${SIMAPP_PREFIX}/deepracer-simapp:${SIMAPP_VERSION}-cpu"
SIMAPP_IMG="${REPO}/deepracer-on-aws-simapp:${DROA_VERSION}"
if [ "$(docker images -q ${SIMAPP_BUILD_IMG} 2>/dev/null)" == "" ] || [ -n "${OPT_NOCACHE}" ]; then
    echo "SimApp image ${SIMAPP_BUILD_IMG} not found - running build.sh..."
    "$DIR/build.sh" -a cpu -p "${SIMAPP_PREFIX}" ${OPT_NOCACHE:+-f}
else
    echo "SimApp image ${SIMAPP_BUILD_IMG} already exists, skipping build."
fi
docker tag "${SIMAPP_BUILD_IMG}" "${SIMAPP_IMG}"

# --- Model Validation ---
echo "Building model-validation image..."
docker buildx build . ${OPT_NOCACHE} \
    -t ${REPO}/deepracer-on-aws-model-validation:${DROA_VERSION} \
    -f docker/Dockerfile.model-validation \
    --build-arg DROA_VERSION=${DROA_VERSION}

# --- Reward Function Validation ---
echo "Building reward-function-validation image..."
docker buildx build . ${OPT_NOCACHE} \
    -t ${REPO}/deepracer-on-aws-reward-function-validation:${DROA_VERSION} \
    -f docker/Dockerfile.reward-function-validation

echo "Done. Built images:"
echo "  ${REPO}/deepracer-on-aws-simapp:${DROA_VERSION}"
echo "  ${REPO}/deepracer-on-aws-model-validation:${DROA_VERSION}"
echo "  ${REPO}/deepracer-on-aws-reward-function-validation:${DROA_VERSION}"
