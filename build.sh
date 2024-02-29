#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
    echo "Requested to stop."
    exit 1
}

set -e

PREFIX="awsdeepracercommunity"
ARCH="cpu-avx cpu-avx2 gpu"

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

    if [ -n "$(echo $a | gawk '/cpu/')" ]; then
        if [ -n "$(echo $a | gawk '/cpu-gl/')" ]; then
            arch_primary="cpu-gl"
        else
            arch_primary="cpu"
        fi

        arch_secondary="$(echo $a | gawk 'match($0, /(cpu)(-gl)?-(.*)/, m) { print m[3] }')"
        arch_tag="$arch_primary-$arch_secondary"

    elif [ -n "$(echo $a | gawk '/gpu/')" ]; then
        arch_primary=$a
        arch_tag=$a

    else
        echo "Architecture $a unknown."
    fi

    set -x
    docker buildx build . ${OPT_NOCACHE} -t $PREFIX/deepracer-robomaker:${VERSION}-${arch_tag} -f docker/Dockerfile.${arch_primary} --build-arg IMG_VERSION=$VERSION --build-arg BUNDLE_PREFIX=${PREFIX}
    set +x

done
