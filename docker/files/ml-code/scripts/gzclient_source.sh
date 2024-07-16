if [ ! -z $GAZEBO_MASTER_URI ]; then
    tmp_GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI
fi

source /opt/ros/noetic/setup.bash
source /usr/share/gazebo-11/setup.sh

if [ ! -z $tmp_GAZEBO_MASTER_URI ]; then
    export GAZEBO_MASTER_URI=$tmp_GAZEBO_MASTER_URI
    unset tmp_GAZEBO_MASTER_URI
fi