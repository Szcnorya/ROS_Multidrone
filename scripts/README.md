<!-- NOTE: Outdated Now for real drone !
     2019/2/14
 -->

What these scripts do:
launch_sim.sh takes in number of drones as cmdline parameter and start the individual components of the simulation (launch gazebo, start ros nodes, and have drones communicate between ROS and Gazebo).

launch_sim.sh delegates to individual scripts for starting gazebo (start_gazebo.sh) and running the drones separately (start_ros_nodes.sh and start_drones.sh).
1. Launch gazebo simulation (https://dev.px4.io/en/simulation/ros_interface.html)
     - Associated script under ROS_Sim/scripts, run with ./start_gazebo.sh [firmware_location]
     
     This script does the following:
     - After opening terminal, ensure ROS env variables are set up. this includes the px4 Firmware directory
     ```
     source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
     export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
     ```
     - Launch the simulation
     ```roslaunch px4 multi_uav_mavros_sitl.launch```
     
     - Starting position of the drones can be changed in the launch file (generate_model.pl also controls this)
     ```
     /src/Firmware/launch/multi_uav_mavros_sitl.launch
     ```     
2. Run drones
     In a new terminal:
     - Run scripts (ros_sim_dir is where ROS_Sim is cloned, including ROS_Sim directory name. ie: ~/projects/ROS_Sim)
     ```
     cd ROS_Sim/scripts
     ./start_ros_nodes.sh [ros_sim_dir]
     
     in new terminal:
     cd ROS_Sim/scripts
     ./start_drones.sh [ros_sim_dir]
     ```
     
     These scripts do the following:
     - Set up environment variables so ROS can find this repo's packages
     ```
     source ROS_Sim/ros_px4_multi/testnavi/devel/setup.bash
     ```
     
     - Spawns ROS nodes that send mavlink messages
     ```
     cd ROS_Sim/ros_px4_multi/testnavi/src/drone/scripts
     rosrun drone DroneRun.py <drone count>
     ```
    
     - In a different terminal, set up env variables so ROS can find this repo's packages
     ```
     source ROS_Sim/ros_px4_multi/testnavi/devel/setup.bash
     ```
     
     - Have nodes start communicating with gazebo simulator
     ```
     rosrun navi navi <drone count>
     ```

Changing drone parameters:
- Associated script: ./generate_model.pl [num_drones] [starting_port]
- Adding/modifying drone models is taken care of by generate_model.pl, and the following are for reference for future changes to drone parameters.

Modifying drones:
- Everything in src is gazebo code
```
cd ~/src/Firmware/posix-configs/SITL/init/ekf2
```

- Create new drone, ie:
```
cp iris_2 iris_3
```

- Change the following params in iris_3:
```
MAV_SYS_ID //increment by 1
SITL_UDP_PORT //increment by 2
mavlink start -x -u # //increment both of these lines by 2
-m onboard -o # //increment by 2
anything that starts with mavlink, inc by 2
```
https://dev.px4.io/en/simulation/multi-vehicle-simulation.html

- Add new UAV to launch file and create new bindings for UDP ports
Edit ~/src/Firmware/launch/multi_uav_mavros_sitl.launch
```
cp -r navi_2 navi_3
```
Edit all files in ROS_Sim/rox_px4_multi/testnavi/src/navi_3
Change all references of 'uav2' to 'uav3', 'navi_2' -> 'navi_3'
