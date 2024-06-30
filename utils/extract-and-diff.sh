#!/bin/bash
set -e 
git checkout upstream

mkdir -p /tmp/bundle-src


docker pull public.ecr.aws/k1d3r4z1/deepracer-sim-public:latest
docker run -d -i --rm --name deepracer-simapp public.ecr.aws/k1d3r4z1/deepracer-sim-public bash
# docker export deepracer-simapp | tar xf - opt/amazon --exclude=/opt/amazon/build --exclude=/opt/amazon/install --exclude=/opt/amazon/log
docker cp deepracer-simapp:/opt/amazon /tmp/bundle-src
docker stop deepracer-simapp

cd /tmp/bundle-src
cd opt/amazon
rm -rf build/ install/ log/

cp -r * ~/deepracer-simapp/bundle/

