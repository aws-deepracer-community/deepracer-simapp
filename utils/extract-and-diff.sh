#!/bin/bash
set -e

IMAGE_TAG="v1.0.3"

usage() {
	echo "Usage: $0 [-t <tag>]"
	echo "  -t, --tag   Docker image tag (default: ${IMAGE_TAG})"
}

while [[ $# -gt 0 ]]; do
	case "$1" in
		-t|--tag)
			IMAGE_TAG="$2"
			shift 2
			;;
		-h|--help)
			usage
			exit 0
			;;
		*)
			echo "Unknown option: $1"
			usage
			exit 1
			;;
	esac
done

git checkout upstream-ros2

mkdir -p /tmp/bundle-src

docker pull public.ecr.aws/aws-solutions/deepracer-on-aws-simapp:${IMAGE_TAG}
docker run -d -i --rm --name deepracer-simapp public.ecr.aws/aws-solutions/deepracer-on-aws-simapp:${IMAGE_TAG} bash
docker cp deepracer-simapp:/opt/amazon /tmp/bundle-src
docker cp deepracer-simapp:/opt/ml /tmp/bundle-src
docker stop deepracer-simapp

cd /tmp/bundle-src/amazon
rm -rf build/ install/ log/

rsync -a --delete ./ ~/deepracer-simapp/bundle/

cd /tmp/bundle-src/ml/code
rm -rf python3.10/
rsync -a --delete ./ ~/deepracer-simapp/docker/files/ml-code/

