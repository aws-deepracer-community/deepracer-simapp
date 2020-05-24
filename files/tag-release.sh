#!/bin/bash
VERSION="2.0.6"
SOURCE_TAG=$(docker images | grep $VERSION | awk '/deepracer-robomaker/ {print $2}')
PREFIX="awsdeepracercommunity/deepracer-robomaker:"
for TAG in $SOURCE_TAG; do
    DEST_TAG=$(echo $TAG | cut -f2- -d\- )
    docker tag ${PREFIX}${TAG} ${PREFIX}${DEST_TAG} 
done
docker tag ${PREFIX}cpu-avx ${PREFIX}cpu