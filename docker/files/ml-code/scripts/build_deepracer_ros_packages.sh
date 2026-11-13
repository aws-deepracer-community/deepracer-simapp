#!/bin/bash

#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

set -e
set -x

echo "Building DeepRacer ROS packages"

ROS_DISTRO=jazzy
ROOT_DIR=/opt/amazon
export ROS_VERSION=2

cd $ROOT_DIR
. /opt/ros/$ROS_DISTRO/setup.bash

echo "Installing ros dependencies"
apt-get update && rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Ensure AWS SDK libraries are in the library path
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ldconfig

rm -rf build/ install/
echo "Doing colcon build"
# Build core packages first
colcon build --packages-select aws_common aws_ros2_common --parallel-workers 1 --cmake-args "-DSETUPTOOLS_DEB_LAYOUT=OFF" "-DPYTHON_EXECUTABLE=/usr/bin/python3" "-DPYTHON_INCLUDE_DIR=/usr/include/python3.12"

# Build the rest with retry logic for intermittent failures
for i in {1..3}; do
    echo "Build attempt $i"
    if colcon build --parallel-workers 1 --cmake-args "-DSETUPTOOLS_DEB_LAYOUT=OFF" "-DPYTHON_EXECUTABLE=/usr/bin/python3" "-DPYTHON_INCLUDE_DIR=/usr/include/python3.12"; then
        echo "Build successful on attempt $i"
        break
    else
        echo "Build failed on attempt $i"
        if [ $i -eq 3 ]; then
            echo "All build attempts failed"
            exit 1
        fi
        echo "Retrying in 5 seconds..."
        sleep 5
        # Clean up partial build artifacts
        rm -rf build/ install/
    fi
done
