#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
    echo "Requested to stop."
    exit 1
}

PREFIX="awsdeepracercommunity"

while getopts ":cgfp:" opt; do
    case $opt in
    c)
        OPT_CORE="core"
        ;;
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

if [ -n "${OPT_CORE}" ]; then
    echo "Preparing core builder image..."
    docker buildx build ${OPT_NOCACHE} -t ${PREFIX}/deepracer-robomaker-build-core:latest -f docker/Dockerfile.build-core .
fi

echo "Preparing devel image for user $(id -u)..."
docker buildx build ${OPT_NOCACHE} -t ${PREFIX}/deepracer-robomaker-build-devel:latest -f docker/Dockerfile.localuser --build-arg FROM_IMG=${PREFIX}/deepracer-robomaker-build-core:latest --build-arg USERNAME=$USER --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) .

echo "Building development build of bundle into $(pwd)/bundle..."
docker run --rm -ti -v $(pwd)/bundle:/opt/bundle ${PREFIX}/deepracer-robomaker-build-devel:latest bash -c 'colcon build'

if [ -n "${OPT_GAZEBO}" ]; then
    if [ -z "${DR_ROBOMAKER_IMAGE}" ]; then
        echo "No base image specified, please provide via DR_ROBOMAKER_IMAGE".
        exit 1
    fi

    if [ -z "${DR_WORLD_NAME}" ]; then
        echo "No world name specified, please provide via DR_WORLD_NAME".
        exit 1
    fi

    echo "Starting Gazebo using awsdeepracercommunity/deepracer-robomaker:${DR_ROBOMAKER_IMAGE} with world ${DR_WORLD_NAME}."
    USER_UID=$(id -u) USER_GID=$(id -g) docker compose -f docker/docker-compose-development.yml up --build
fi
