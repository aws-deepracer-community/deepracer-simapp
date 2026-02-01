#!/bin/bash
set -e 
git checkout upstream

mkdir -p /tmp/bundle-src

docker pull public.ecr.aws/aws-solutions/deepracer-on-aws-simapp:v1.0.0
docker run -d -i --rm --name deepracer-simapp public.ecr.aws/aws-solutions/deepracer-on-aws-simapp:v1.0.0 bash
docker cp deepracer-simapp:/opt/amazon /tmp/bundle-src
docker cp deepracer-simapp:/opt/ml /tmp/bundle-src
docker stop deepracer-simapp

cd /tmp/bundle-src/amazon
rm -rf build/ install/ log/

cp -r * ~/deepracer-simapp/bundle/

