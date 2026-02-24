#! /usr/bin/env bash

# the following variable will only be set by AWS RoboMaker, when run in AWS RoboMaker
# AWS_ROBOMAKER_WORLDFORGE_WORLD_PACKAGE_OVERRIDE
# AWS_ROBOMAKER_WORLDFORGE_SETUP_OVERRIDE

if [[ -n "${AWS_ROBOMAKER_WORLDFORGE_WORLD_PACKAGE_OVERRIDE}" ]]; then
    # buffer and reset _colcon_prefix_sh_source_script as sourcing setup.sh will unset this function variable
    COLCON_PREFIX_BUFFER_FN=$(declare -f $_colcon_prefix_sh_source_script)
    BUNDLE_CURRENT_PREFIX=${AWS_ROBOMAKER_WORLDFORGE_SETUP_OVERRIDE:-/opt/robomaker/worldforge/$ROS_DISTRO}
    source "${BUNDLE_CURRENT_PREFIX}"/setup.sh
    unset BUNDLE_CURRENT_PREFIX
    eval "$COLCON_PREFIX_BUFFER_FN"
    unset COLCON_PREFIX_BUFFER_FN
fi