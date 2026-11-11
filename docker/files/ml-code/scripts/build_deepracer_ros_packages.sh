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

ROS_DISTRO=lyrical
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
# One build of all packages. The earlier two-phase build with three retries probably hid an
# intermittent link failure: aws_common and kinesis_webrtc_manager installed a second
# <package>Config.cmake over the one from ament (fixed in AwsSilverstoneROSKvsStreamer), so a
# failure here is a real failure.
# Parallelism: COLCON_WORKERS packages at a time, each with MAKE_JOBS make jobs. Set both to
# match the build host's cores and memory.
export MAKEFLAGS="-j${MAKE_JOBS:-4}"
# CMake 4 (Ubuntu 26.04) rejects cmake_minimum_required below 3.5. The KVS producer and WebRTC
# SDKs that kinesis_manager and kinesis_webrtc_manager download and build at build time (and their
# bundled open-source dependencies) still declare older minimums; this lets them configure with
# the 3.5 policies. Our own CMakeLists.txt files declare 3.10 or newer.
export CMAKE_POLICY_VERSION_MINIMUM=3.5
colcon build --parallel-workers "${COLCON_WORKERS:-2}" --event-handlers console_cohesion+ \
    --cmake-args "-DSETUPTOOLS_DEB_LAYOUT=OFF" "-DPYTHON_EXECUTABLE=/usr/bin/python3" \
    "-DPYTHON_INCLUDE_DIR=/usr/include/python3.14"

# The colcon build tree (about 1.5 GB, including the downloaded KVS SDK sources) is not needed at
# run time; leaving it in the image layer only adds size. Set KEEP_COLCON_BUILD=1 to keep it, for
# example to run `colcon test`.
if [ "${KEEP_COLCON_BUILD:-0}" != 1 ]; then
    rm -rf build/ log/
fi
