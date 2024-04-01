#!/bin/bash
#
# Setup environment to make PX4 visible to ROS


PX4_PATH=~/PX4-Autopilot

# setup ROS env and update package path
source $PX4_PATH/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4_PATH $PX4_PATH/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_PATH:$PX4_PATH/Tools/simulation/gazebo-classic/sitl_gazebo_classic

echo -e "ROS_PACKAGE_PATH $ROS_PACKAGE_PATH"