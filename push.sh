#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
    echo "Requested to stop."
    exit 1
}

PREFIX="local"
VERSION=$(cat VERSION)

while getopts ":a:p:" opt; do
    case $opt in
    a)
        ARCH="$OPTARG"
        ;;
    p)
        PREFIX="$OPTARG"
        ;;
    \?)
        echo "Invalid option -$OPTARG" >&2
        exit 1
        ;;
    esac
done

echo "Pushing docker images for [$ARCH]"

for A in $ARCH; do
    echo "Pushing $PREFIX/deepracer-robomaker:$VERSION-$A"
    docker push $PREFIX/deepracer-robomaker:$VERSION-$A
done
