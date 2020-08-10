#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
        echo "Requested to stop."
        exit 1
}

PREFIX="awsdeepracercommunity"
ARCH="cpu-avx cpu-avx2 cpu-gl-avx cpu-gl-avx2 gpu gpu-gl"
TF_PATH='https://larsll-build-artifact-share.s3.eu-north-1.amazonaws.com/tensorflow/${arch_tf}/tensorflow-1.12.3-cp35-cp35m-linux_x86_64.whl'

while getopts ":a:fp:t:" opt; do
case $opt in
a) ARCH="$OPTARG"
;;
p) PREFIX="$OPTARG"
;;
t) TF_PATH="$OPTARG"
;;
f) OPT_NOCACHE="--no-cache"
;;
\?) echo "Invalid option -$OPTARG" >&2
exit 1
;;
esac
done

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
VERSION=$(cat $DIR/VERSION)

echo "Preparing docker images for [$ARCH]"

for a in $ARCH; do   
    arch_p1=$(echo $a | cut -f1 -d- )
    if [ "$arch_p1" == "cpu" ]; then
        arch_p2=$(echo $a | cut -f2 -d- )
        if [ "$arch_p2" == "gl" ]; then
            arch_tf=$(echo $a | cut -f3 -d- )
            tf=$(eval echo $TF_PATH)
            docker build . -t $PREFIX/deepracer-robomaker:${VERSION}-cpu-gl-${arch_tf} -f docker/Dockerfile.cpu-gl --build-arg TENSORFLOW_VER=$tf --build-arg IMG_VERSION=$VERSION
        else
            arch_tf=$arch_p2
            tf=$(eval echo $TF_PATH)
            docker build . -t $PREFIX/deepracer-robomaker:${VERSION}-cpu-${arch_p2} -f docker/Dockerfile.cpu --build-arg TENSORFLOW_VER=$tf --build-arg IMG_VERSION=$VERSION
        fi 
    elif [ "$arch_p1" == "gpu" ]; then
        docker build . -t $PREFIX/deepracer-robomaker:${VERSION}-${a} -f docker/Dockerfile.${a} --build-arg IMG_VERSION=$VERSION
    fi
done
