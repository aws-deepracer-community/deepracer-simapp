#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
    echo "Requested to stop."
    exit 1
}

PREFIX="awsdeepracercommunity"

while getopts ":gfp:" opt; do
    case $opt in
    p)
        PREFIX="$OPTARG"
        ;;
    g)
        OPT_GAZEBO="gazebo"
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

# On Windows/Git Bash $USER is not set; fall back to whoami
if [ -z "$USER" ]; then
    USER=$(whoami)
fi

if [ "$(docker images -q ${PREFIX}/deepracer-env-build-core:latest 2> /dev/null)" == "" ] || [ -n "${OPT_NOCACHE}" ]; then
    echo "Preparing core builder image ${PREFIX}/deepracer-env-build-core:latest..."
    docker buildx build ${OPT_NOCACHE} -t ${PREFIX}/deepracer-env-build-core:latest -f docker/Dockerfile.base .
else
    echo "Core builder image ${PREFIX}/deepracer-env-build-core:latest already exists."
fi

echo "Preparing devel image for user $(id -u)..."
docker buildx build ${OPT_NOCACHE} -t ${PREFIX}/deepracer-env-build-devel:latest -f docker/Dockerfile.dev-user \
    --build-arg FROM_IMG=${PREFIX}/deepracer-env-build-core:latest --build-arg USERNAME=$USER --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) .

echo "Building development build of bundle into $(pwd)/install..."
mkdir -p $(pwd)/install $(pwd)/build $(pwd)/log
docker run --rm -ti -v $(pwd)/simulation:/opt/bundle -v $(pwd)/log:/opt/log -v $(pwd)/install:/opt/simapp -v $(pwd)/build:/opt/build \
    ${PREFIX}/deepracer-env-build-devel:latest bash -c 'sudo mkdir -p /opt/log /opt/simapp /opt/build && sudo chown -R $(whoami) /opt/log /opt/simapp /opt/build && cd /opt/bundle && colcon --log-base /opt/log build --install-base /opt/simapp --build-base /opt/build'

if [ -n "${OPT_GAZEBO}" ]; then
    if [ -z "${DR_SIMAPP_IMAGE}" ]; then
        echo "No base image specified, please provide via DR_SIMAPP_IMAGE".
        exit 1
    fi

    if [ -z "${DR_WORLD_NAME}" ]; then
        echo "No world name specified, please provide via DR_WORLD_NAME".
        exit 1
    fi

    echo "Starting Gazebo using awsdeepracercommunity/deepracer-env:${DR_SIMAPP_IMAGE} with world ${DR_WORLD_NAME}."
    USER_UID=$(id -u) USER_GID=$(id -g) docker compose -f docker/docker-compose-development.yml up
fi
