#!/bin/bash
mkdir -p /tmp/bundle-src

git checkout upstream

docker pull public.ecr.aws/k1d3r4z1/deepracer-sim-public:latest
docker run -d -i --rm --name deepracer-simapp public.ecr.aws/k1d3r4z1/deepracer-sim-public bash
docker export -o /tmp/bundle-src/deepracer-simapp-`date +%Y-%m-%d`.tar deepracer-simapp 
docker stop deepracer-simapp

cd /tmp/bundle-src
tar xf deepracer-simapp-`date +%Y-%m-%d`.tar opt/amazon
cd opt/amazon
rm -rf build/ install/ log/

cp -r * ~/deepracer-simapp/bundle/

