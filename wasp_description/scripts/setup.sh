#!/bin/bash
#
# Setup environment variables to handle distributed ROS network

echo "xxxxxxxxxxxxxxxxxxxxxx"
echo $1
echo $2
echo $3
echo $4

echo ip addr show wlp0s20f3|grep inet|grep -v inet6|awk '{print $2}'|awk '{split($0,a,"/"); print a[1]}'
echo "test"
echo -e $1
echo $2
n=101


if [[ $1 == 0 ]]
then # 0 -> Master
  echo "The number is even."
else # !0 -> Wasp ID
  echo "The number is odd."

#   export ROS_IP
fi


# PX4_PATH=~/PX4-Autopilot
#
# # setup ROS env and update package path
# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_PATH
# source $PX4_PATH/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4_PATH $PX4_PATH/build/px4_sitl_default
#
# echo -e "ROS_PACKAGE_PATH $ROS_PACKAGE_PATH"
