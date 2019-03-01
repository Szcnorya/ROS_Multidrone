/*-----------------------------------------------------------------------------

                                                         Author: Jason Ma
                                                                 David Yang
                                                                 Michael Chen
                                                         Date:   Sep 04 2018
                              ROS PX4 Multi-drone

  File Name:      navi.cpp
  Description:    Navigation controller for a variable number of drones. Acts
                  as interface between DroneModule.py and PX4/mavros. The
                  waypoints/velocities published here are what drones follow.
-----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h> 
#include <mavros_msgs/State.h> 
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>

// get state of drone 
mavros_msgs::State current_state;
void state_callback(const mavros_msgs::State::ConstPtr& msg) {
  current_state = *msg;
}

// get next target msg from decision code 
sensor_msgs::NavSatFix pos_next;
void target_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  pos_next = *msg;
}

// get current lcoation 
sensor_msgs::NavSatFix pos_gps;
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  pos_gps = *msg;
}

// get landing signal
std_msgs::Bool land_signal;
void land_signal_callback(const std_msgs::Bool::ConstPtr& msg) {
  land_signal = *msg;
}

geometry_msgs::Twist vel_target_python;
void vel_target_callback(const geometry_msgs::Twist::ConstPtr& msg) {
  vel_target_python = *msg;
}

//[Global vars]----------------------------------------------------------------


mavros_msgs::GlobalPositionTarget pos_target;
mavros_msgs::SetMode mode_guided;
mavros_msgs::CommandBool cmd_arm;
mavros_msgs::CommandTOL cmd_takeoff;
mavros_msgs::CommandTOL cmd_land;

geometry_msgs::TwistStamped vel_target_stamped;

// Subscribers
ros::Subscriber sub_state;
ros::Subscriber sub_obj;
ros::Subscriber sub_gps;
ros::Subscriber sub_vel_ref;
//ros::Subscriber sub_signal;

// Publishers
ros::Publisher pub_pos;
ros::Publisher pub_vel;
//ros::Publisher pub_vel_ang;

// Clients
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
//ros::ServiceClient takeoff_client;
ros::ServiceClient landing_client;

/*-----------------------------------------------------------------------------
  Routine Name: pub_lin_vel
  File:         navi.cpp
  
  Description: Publishes linear velocity vector
  
  Parameter Descriptions:
  name               description
  ------------------ -----------------------------------------------
  vel_lin            vector representing velocities for x, y, z
-----------------------------------------------------------------------------*/
void pub_lin_vel(geometry_msgs::Twist vel_target) {
  geometry_msgs::Vector3 vel_lin_target;
  geometry_msgs::Vector3 vel_ang_target;

  vel_target_stamped.header.stamp = ros::Time::now();
  vel_target_stamped.header.seq++;

  vel_target_stamped.twist = vel_target;

  pub_vel.publish(vel_target_stamped);
}


/*-----------------------------------------------------------------------------
  Routine Name: pub_lin_vel
  File:         navi.cpp
  
  Description: Publishes linear velocity vector
  
  Parameter Descriptions:
  name               description
  ------------------ -----------------------------------------------
  x                  x velocity
  y                  y velocity
  z                  z velocity (altitude)
-----------------------------------------------------------------------------*/
void pub_lin_vel(double x, double y, double z) {
  geometry_msgs::Twist vel_target;
  geometry_msgs::Vector3 vel_lin_target;
  geometry_msgs::Vector3 vel_ang_target;

  vel_target_stamped.header.stamp = ros::Time::now();
  vel_target_stamped.header.seq++;

  vel_lin_target.x = x;
  vel_lin_target.y = y;
  vel_lin_target.z = z;

  vel_ang_target.x = 0;
  vel_ang_target.y = 0;
  vel_ang_target.z = 0; //yaw
  
  vel_target.linear = vel_lin_target;
  vel_target.angular = vel_ang_target;
  vel_target_stamped.twist = vel_target;

  pub_vel.publish(vel_target_stamped);
}


/*-----------------------------------------------------------------------------
  Routine Name: range_calc
  File:         navi.cpp
  
  Description: Calculates distance between two gps coordinates in meters
  
  Parameter Descriptions:
  name               description
  ------------------ -----------------------------------------------
  lat/lon/alt        first set of gps coords
  t_lat/t_lon/t_alt  second set of gps coords
  return             distance between gps coordinates
-----------------------------------------------------------------------------*/
double range_calc(float lat, float lon, float alt, float t_lat, float t_lon, float t_alt) {
  return pow(pow((lat - t_lat) * 111111.11, 2) + pow((lon - t_lon) * 111111.11,2) + pow(alt - t_alt, 2), 0.5);
}

/*-----------------------------------------------------------------------------
  Routine Name: main
  File:         navi.cpp
  
  Description: Multi-process program to spin up ROS nodes and control them as
               a swarm.
  
  Parameter Descriptions:
  name               description
  ------------------ -----------------------------------------------
  argc               argument count
  argv               argument vector
-----------------------------------------------------------------------------*/
int main(int argc, char **argv) {
  // Let the module accept its ID from argument
  int Drone_id = std::stoi(argv[1]);
  if(Drone_id < 0){
    return 1;
  }
  //const char command []= "rosrun drone DroneRun.py %put drone count% &";
  //system(command);
  //havn't tested this^^ yet but it can potentially make this the 
  //only thing user need to manually execute

  // Create names under namespace for individual drone
  std::string nodename        = "px4_navi_" + std::to_string(Drone_id);
  std::string group_ns        = "uav" + std::to_string(Drone_id);

  // Create ros node
  ros::init(argc, argv, nodename);
  ros::NodeHandle nh;
  ros::Rate rate(20.0);

  // Subscriber topic names
  std::string sub_state_str   = group_ns + "/mavros/state";
  std::string sub_vel_ref_str = group_ns + "/vel_ref";
  std::string sub_gps_str     = group_ns + "/mavros/global_position/global";

  // Publisher topic names
  std::string pub_pos_str     = group_ns + "/mavros/setpoint_position/global";
  std::string pub_vel_str     = group_ns + "/mavros/setpoint_velocity/cmd_vel";
  //std::string ang_vel_pub_s = groupNS + "/mavros/setpoint_attitude/cmd_vel";

  // Client topic names
  std::string cli_arming_str  = group_ns + "/mavros/cmd/arming";
  std::string cli_mode_str    = group_ns + "/mavros/set_mode";
  std::string cli_land_str    = group_ns + "/mavros/cmd/land";

  // Subscribers
  sub_state = nh.subscribe<mavros_msgs::State>(sub_state_str, 10, state_callback);
  //sub_obj = nh.subscribe<sensor_msgs::NavSatFix>(sub_obj_str, 10, target_callback); //geo msg
  sub_gps = nh.subscribe<sensor_msgs::NavSatFix>(sub_gps_str, 10, gps_callback); //geo msg
  sub_vel_ref = nh.subscribe<geometry_msgs::Twist>(sub_vel_ref_str, 10, vel_target_callback);
  //sub_signal = nh.subscribe<std_msgs::Bool>("land_sig", 10, land_signal_callback);

  // Publishers
  pub_pos = nh.advertise<mavros_msgs::GlobalPositionTarget>(pub_pos_str, 10);
  pub_vel = nh.advertise<geometry_msgs::TwistStamped>(pub_vel_str, 10);
  //pub_vel_ang = nh.advertise<geometry_msgs::Twist>(pub_vel_ang_str, 10);

  // Clients
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>(cli_arming_str);
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(cli_mode_str);
  //takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("uav1/mavros/cmd/takeoff");
  landing_client = nh.serviceClient<mavros_msgs::CommandTOL>(cli_land_str);


  //[Drone logic start]--------------------------------------------------------
  /*TODO check sensors status NOT IMPLEMENTED*/
  /*TODO check battery status NOT IMPLEMENTED*/
  //battery state msg in mavros
  
  // Wait until ROS node spins up
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }
 
  double alt_home = pos_gps.altitude;
  
  // Set initial target position to 30 meters above home position
  pos_target.latitude  = pos_gps.latitude;
  pos_target.longitude = pos_gps.longitude;
  pos_target.altitude  = pos_gps.altitude + 30;

  // Publish init target to stream so offboard doesn't shut down immediately
  for(int i = 100; ros::ok() && i > 0; --i){
    //pub_pos.publish(pos_target);
    pub_lin_vel(0, 0, 5);
    ros::spinOnce();
    rate.sleep();
  }
  
  // Set px4 mode to offboard and begin takeoff sequence if on ground
  mode_guided.request.custom_mode = "OFFBOARD";
  cmd_arm.request.value = true;
  cmd_takeoff.request.altitude = 15;

  bool at_target = false;
  bool alt_reached = false;
  int debugger = 0;
  ros::Time last_request = ros::Time::now();
  
  // Loop while ROS is online
  while(ros::ok()){
    //handle takeoff sequence
    if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
      if(set_mode_client.call(mode_guided) && mode_guided.response.mode_sent) {
        ROS_INFO("[debug] Offboard enabled");
      }
      last_request = ros::Time::now();
    }
    else {
      if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        pos_target.latitude  = pos_gps.latitude;
        pos_target.longitude = pos_gps.longitude;
        pos_target.altitude  = pos_gps.altitude + 30;
        alt_home = pos_gps.altitude;
        at_target = false;

        if(arming_client.call(cmd_arm) && cmd_arm.response.success) {
          ROS_INFO("[debug] Vehicle armed [%f %f %f]", pos_target.latitude, pos_target.longitude, pos_target.altitude);
        }
        last_request = ros::Time::now();
      }
    }

    /*TODO:Landing not implemented, though px4 auto returns home if setpoint stream is cut*/
    if(land_signal.data){
    
      /*can use landing state in topic extended state?*/
      /*perform service call to land*/
      if( landing_client.call(cmd_land) && cmd_land.response.success){
        ROS_INFO("drone is landing");
        sleep(100);
      }else{
        ROS_INFO("landing FAILED");
        
        /*not sure what to do here*/
        //exit(EXIT_FAILURE);
      }
    
      exit(0);
    }
    
    //check whether at destination
    if(range_calc(pos_gps.latitude, pos_gps.longitude, pos_gps.altitude, pos_target.latitude, pos_target.longitude, pos_target.altitude) < 5){
      at_target = true;
    }

    //if at target, set new target waypoint
    if(at_target){
      //target_.latitude=nextT_.latitude;
      //target_.longitude=nextT_.longitude;
      //target_.altitude=nextT_.altitude;
      alt_reached = true;
      //ROS_INFO("Next target set %f, %f ,%f", target_.latitude, target_.longitude,target_.altitude);
      at_target = false;
    }

    if(alt_reached) {
      pub_lin_vel(vel_target_python);
    }
    else {
      pub_lin_vel(0, 0, 5);
      //pub_pos.publish(pos_target);
    }
    //ang_vel_pub.publish(target_vel);
    ros::spinOnce();
    rate.sleep();
  
  }
  return 0;
}
