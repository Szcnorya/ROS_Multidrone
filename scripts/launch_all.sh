export ROS_SIM_DIR=~/projects/ROS_Sim
export DRONE_ID=$1

if [ -z "$FIRMWARE_DIR" ];
then
	echo "No PX4 firmware directory specified"
	echo "usage: ./launch_all.sh [DRONE_ID]"
	exit 1
fi

if [ -z "$ROS_SIM_DIR" ];
then
	echo "No ROS_Sim base dir specified"
	echo "usage: ./launch_all.sh [DRONE_ID]"
fi

if [ -z "$DRONE_ID" ];
then
	echo "No drone ID specified"
	echo "usage: ./launch_all.sh [DRONE_ID]"
fi

echo "Drone ID: ${DRONE_ID}"

bash -c "${ROS_SIM_DIR}/scripts/start_ros_nodes.sh ${ROS_SIM_DIR} ${NUM_DRONES}; exec bash" &
bash -c "${ROS_SIM_DIR}/scripts/start_drones.sh ${ROS_SIM_DIR} ${NUM_DRONES}; exec bash" &