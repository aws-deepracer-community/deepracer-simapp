#! /usr/bin/env bash

GAZEBO_EXE_NAME=$1
WORLD_SDF_PATH=$2
OTHER_ARGS=${@:3}

# the following variables will only be set by AWS RoboMaker, when run in AWS RoboMaker
# AWS_ROBOMAKER_WORLDFORGE_WORLD_PACKAGE_OVERRIDE

if [[ -n "${AWS_ROBOMAKER_WORLDFORGE_WORLD_PACKAGE_OVERRIDE}" ]]; then
    # older exported worldforge worlds will not have the environment hooks to enable this
    worldforge_world_pkg_count=`echo $ROS_PACKAGE_PATH | grep -o aws_robomaker_worldforge_worlds | wc -l`
    if [ "$worldforge_world_pkg_count" -gt "1" ]; then
        CONFLICTING_WORLD_WARNING="[Wrn] The AWS Robomaker worldforge world referenced in your worldConfig \
        might not be used in the simulation job because it conflicts with a AWS Robomaker worldforge world \
        from your workspace. Please re-export your AWS RoboMaker worldforge world(s) to resolve this issue. \
        This message can be safely ignored if you are using a bundle to create the simulation job."
        echo $CONFLICTING_WORLD_WARNING
    fi
    world_pkg_path=`rospack find aws_robomaker_worldforge_worlds`
    world_file_path=`find "${world_pkg_path}/worlds" -type f -name "generation_*_world_*.world" -print`
    WORLD_SDF_PATH=${world_file_path}
fi

script_path=$(readlink -f "$0")  # path to this script
setup_path=$(pkg-config --variable=prefix gazebo)/share/gazebo/

# source setup.sh, but keep local modifications to GAZEBO_MASTER_URI and GAZEBO_MODEL_DATABASE_URI
desired_master_uri="${GAZEBO_MASTER_URI}"
desired_model_database_uri="${GAZEBO_MODEL_DATABASE_URI}"
desired_model_path="${GAZEBO_MODEL_PATH}"
. ${setup_path}/setup.sh

if [[ -z ${desired_master_uri} ]]; then
    desired_master_uri="${GAZEBO_MASTER_URI}"
fi
if [[ -z ${desired_model_database_uri} ]]; then
    desired_model_database_uri="${GAZEBO_MODEL_DATABASE_URI}"
fi
if [[ -z ${desired_model_path} ]]; then
    desired_model_path="${GAZEBO_MODEL_PATH}"
fi

# the function relocates all the ROS remappings in the command at the end of the
# string this allows some punky uses of rosrun, for more information see:
# https://github.com/ros-simulation/gazebo_ros_pkgs/issues/387
for w in ${OTHER_ARGS}; do
    if $(echo "${w}" | grep -q ':='); then
        ros_remaps="${ros_remaps} ${w}"
    else
        gazebo_args="${gazebo_args} ${w}"
    fi
done

# use the appropriate script from gazebo_ros to launch gazebo
GAZEBO_EXE=$(rospack find gazebo_ros)/../../lib/gazebo_ros/${GAZEBO_EXE_NAME}

# this captures the gazebo model path that WorldForge world package exports
# (via the mechanism explained at https://www.ros.org/reps/rep-0140.html#export)
n=0
for w in $(rospack plugins --attrib=gazebo_model_path gazebo_ros); do
    if [[ $((${n} % 2)) == 1 ]]; then
        if [[ -z ${desired_model_path} ]]; then
            desired_model_path=${w}
        else
            desired_model_path="${desired_model_path}:${w}"
        fi
    fi
    ((n++))
done

GAZEBO_MASTER_URI="${desired_master_uri}" GAZEBO_MODEL_DATABASE_URI="${desired_model_database_uri}" \
    GAZEBO_MODEL_PATH="${desired_model_path}" ${GAZEBO_EXE} ${WORLD_SDF_PATH} ${gazebo_args} ${ros_remaps}
