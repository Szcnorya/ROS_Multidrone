#!/usr/bin/perl


# NOTE: This file need to update to accommodate PATH change
#       Maybe we can only run this file at host
#       The port part can be kept to save effort
# TODO:
#     1. Get rid of all simulator stuff(gazebo dependency)
#     
# High level Syntax(Now): 
#   Generate a single model and launch file for a given ID(they are meant to run on a isolated machine)
# 
# High level Syntax(Previous): 
#   Let all drone have different port number and ID to run on same machine
#   Generate all model files like iris_{droneID} to some folder in /src/Firmware
#   Generate a single launch file at /src/Firmware/launch/multi_uav_mavros_sitl.launch
#   


use strict;
use warnings;

#make sure gazebo files are installed
#in home:~/src/...

#check argument count
if ( @ARGV != 2 ) {
    print
"Error please include 2 arguments\n test.pl <drone ID> <starting port>\n";
    exit;
}

#check argument is positive integer
if ( $ARGV[0] !~ /^[1-9][0-9]*$/ || $ARGV[1] !~ /^[1-9][0-9]*$/ ) {
    print "Arugument need to be a positive integer\n";
    exit;
}

#initialize
#my $fileNum = 1;
my $portNum = $ARGV[1];
my $directoryPath =
  $ENV{"HOME"} . '/src/Firmware/posix-configs/SITL/init/ekf2/';
my $filePrefix = 'iris_';
my $droneID   = $ARGV[0];
my $fileName;    # = $filePrefix . 1;
my $str = "uorb start
param load
dataman start
param set MAV_SYS_ID ${droneID}
param set BAT_N_CELLS 3
param set CAL_ACC0_ID 1376264
param set CAL_ACC0_XOFF 0.01
param set CAL_ACC0_XSCALE 1.01
param set CAL_ACC0_YOFF -0.01
param set CAL_ACC0_YSCALE 1.01
param set CAL_ACC0_ZOFF 0.01
param set CAL_ACC0_ZSCALE 1.01
param set CAL_ACC1_ID 1310728
param set CAL_ACC1_XOFF 0.01
param set CAL_GYRO0_ID 2293768
param set CAL_GYRO0_XOFF 0.01
param set CAL_MAG0_ID 196616
param set CAL_MAG0_XOFF 0.01
param set COM_DISARM_LAND 3
param set COM_OBL_ACT 2
param set COM_OBL_RC_ACT 0
param set COM_OF_LOSS_T 5
param set COM_RC_IN_MODE 1
param set EKF2_AID_MASK 1
param set EKF2_ANGERR_INIT 0.01
param set EKF2_GBIAS_INIT 0.01
param set EKF2_HGT_MODE 0
param set EKF2_MAG_TYPE 1
param set MAV_TYPE 2
param set MC_PITCH_P 6
param set MC_PITCHRATE_P 0.2
param set MC_ROLL_P 6
param set MC_ROLLRATE_P 0.2
param set MIS_TAKEOFF_ALT 2.5
param set MPC_HOLD_MAX_Z 2.0
param set MPC_Z_VEL_I 0.15
param set MPC_Z_VEL_P 0.6
param set NAV_ACC_RAD 2.0
param set NAV_DLL_ACT 2
param set RTL_DESCEND_ALT 5.0
param set RTL_LAND_DELAY 5
param set RTL_RETURN_ALT 30.0
param set SDLOG_DIRS_MAX 7
param set SENS_BOARD_ROT 0
param set SENS_BOARD_X_OFF 0.000001
param set SITL_UDP_PRT ${portNum}
param set SYS_AUTOSTART 4010
param set SYS_MC_EST_GROUP 2
param set SYS_RESTART_TYPE 2
replay tryapplyparams
simulator start -s
tone_alarm start
gyrosim start
accelsim start
barosim start
adcsim start
gpssim start
pwm_out_sim mode_pwm
sensors start
commander start
land_detector start multicopter
navigator start
ekf2 start
mc_pos_control start
mc_att_control start
mixer load /dev/pwm_output0 ROMFS/px4fmu_common/mixers/quad_w.main.mix
mavlink start -x -u ${\($portNum+1)} -r 4000000
mavlink start -x -u ${\($portNum+2)} -r 4000000 -m onboard -o ${\($portNum+3)}
mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u ${\($portNum+1)}
mavlink stream -r 50 -s LOCAL_POSITION_NED -u ${\($portNum+1)}
mavlink stream -r 50 -s GLOBAL_POSITION_INT -u ${\($portNum+1)}
mavlink stream -r 50 -s ATTITUDE -u ${\($portNum+1)}
mavlink stream -r 50 -s ATTITUDE_QUATERNION -u ${\($portNum+1)}
mavlink stream -r 50 -s ATTITUDE_TARGET -u ${\($portNum+1)}
mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0 -u ${\($portNum+1)}
mavlink stream -r 20 -s RC_CHANNELS -u ${\($portNum+1)}
mavlink stream -r 250 -s HIGHRES_IMU -u ${\($portNum+1)}
mavlink stream -r 10 -s OPTICAL_FLOW_RAD -u ${\($portNum+1)}
logger start -e -t
mavlink boot_complete
replay trystart";

#create model files
$fileName = $directoryPath . $filePrefix . ${droneID};
open( FH, '>', $fileName ) or die $!;
print FH $str;
close(FH);

# Generate a single launch file
$fileName = $ENV{"HOME"} . "/src/Firmware/launch/multi_uav_mavros_sitl.launch";
my $launchStr = "<?xml version=\"1.0\"?>
<launch>

    <!-- MAVROS posix SITL environment launch script -->

    <arg name=\"debug\" default=\"false\"/>
    <arg name=\"verbose\" default=\"false\"/>
    <arg name=\"paused\" default=\"false\"/>

    <arg name=\"est\" default=\"ekf2\"/>
    <arg name=\"vehicle\" default=\"iris\"/>
    <arg name=\"world\" default=\"\$(find mavlink_sitl_gazebo)/worlds/empty.world\"/>

    <arg name=\"headless\" default=\"false\"/>
    <arg name=\"gui\" default=\"true\"/>
    <arg name=\"ns\" default=\"/\"/>

    <arg name=\"pluginlists_yaml\" default=\"\$(find mavros)/launch/px4_pluginlists.yaml\" />
    <arg name=\"config_yaml\" default=\"\$(find mavros)/launch/px4_config.yaml\" />

    <!-- Load world -->
    <include file=\"\$(find gazebo_ros)/launch/empty_world.launch\">
        <arg name=\"headless\" value=\"\$(arg headless)\"/>
        <arg name=\"gui\" value=\"\$(arg gui)\"/>
        <arg name=\"world_name\" value=\"\$(arg world)\" />
        <arg name=\"debug\" value=\"\$(arg debug)\" />
        <arg name=\"verbose\" value=\"\$(arg verbose)\" />
        <arg name=\"paused\" value=\"\$(arg paused)\" />
    </include>
";


open( FH, '>', $fileName ) or die $!;
print FH $launchStr;
$groupStr = "
<group ns=\"uav${droneID}\">
    <arg name=\"fcu_url\" default=\"udp://:${\($portNum+3)}\@localhost:${\($portNum+2)}\"/>
    <arg name=\"gcs_url\" value=\"\"/>
    <arg name=\"tgt_system\" value=\"${droneID}\"/> 
    <arg name=\"tgt_component\" value=\"1\"/>
    <arg name=\"rcS${droneID}\" default=\"\$(find px4)/posix-configs/SITL/init/\$(arg est)/\$(arg vehicle)_${droneID}\"/>
    <arg name=\"ID\" value=\"${droneID}\"/>

    <include file=\"\$(find px4)/launch/single_vehicle_spawn.launch\">
        <arg name=\"x\" value=\"${\(($droneID-1) / 4)}\"/>
        <arg name=\"y\" value=\"${\(($droneID-1) % 4)}\"/>
        <arg name=\"z\" value=\"0\"/>
        <arg name=\"R\" value=\"0\"/>
        <arg name=\"P\" value=\"0\"/>
        <arg name=\"Y\" value=\"0\"/>
        <arg name=\"vehicle\" value=\"\$(arg vehicle)\"/>
        <arg name=\"rcS\" value=\"\$(arg rcS${droneID})\"/>
        <arg name=\"mavlink_udp_port\" value=\"${\($portNum)}\"/>
        <arg name=\"ID\" value=\"\$(arg ID)\"/>
    </include>

    <include file=\"\$(find mavros)/launch/node.launch\">
        <arg name=\"pluginlists_yaml\" value=\"\$(arg pluginlists_yaml)\" />
        <arg name=\"config_yaml\" value=\"\$(arg config_yaml)\" />

        <arg name=\"fcu_url\" value=\"\$(arg fcu_url)\" />
        <arg name=\"gcs_url\" value=\"\$(arg gcs_url)\" />
        <arg name=\"tgt_system\" value=\"\$(arg tgt_system)\" />
        <arg name=\"tgt_component\" value=\"\$(arg tgt_component)\" />
    </include>
</group>
"
print FH $groupStr;
print FH "</launch>";
close(FH);