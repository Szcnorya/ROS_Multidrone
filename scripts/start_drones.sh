#!/bin/bash

ROS_SIM_DIR=$1
DRONE_ID=$2

if [ -z "$ROS_SIM_DIR" ];
then
	echo "usage: ./start_drones.sh [ROS_SIM_DIR] [DRONE_ID]"
	exit 1
fi

if [ -z "$DRONE_ID" ];
then
	echo "usage: ./start_drones.sh [ROS_SIM_DIR] [DRONE_ID]"
	exit 1
fi

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

source ${ROS_SIM_DIR}/ros_px4_multi/multidrone/devel/setup.bash
rosrun navi navi ${DRONE_ID}
