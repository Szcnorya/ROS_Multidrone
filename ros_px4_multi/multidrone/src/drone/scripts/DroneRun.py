#!/usr/bin/env python

'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                                 David ?
                                                         Date  : Jul 27 2018
                              ROS Multi-drone Sim

  File Name  : DroneRun.py
  Description: This script sets up all the ROS modules associated with running
               the drone. 
---*-----------------------------------------------------------------------*'''

PKG = 'drone'

import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import math
import numpy
import random
import time
import json

import DroneModule

'''[GLOBAL VARS]------------------------------------------------------------'''
COMM_DROPOFF = 25
#FAKE_GPS_ORIGIN_X = 47.3667
#FAKE_GPS_ORIGIN_Y = 8.5500
#FAKE_GPS_ORIGIN_Z = 408.0

FAKE_GPS_ORIGIN_X = 47.39774
FAKE_GPS_ORIGIN_Y = 8.54561
FAKE_GPS_ORIGIN_Z = 408.0

'''[main]----------------------------------------------------------------------
  Drives program, initializes all modules and drones as well as their ROS
  counterparts.
----------------------------------------------------------------------------'''
if __name__ == "__main__":
  rospy.init_node("DroneTest", anonymous = True)

  # Parse Drone ID from argument
  DroneID = int(sys.argv[1])

  # Desired grid resolution, number of drones, and communication radius (drone-to-drone range)
  resolutions = {'x':40,'y':40,'z':40}

  #Works, but is way out there
  #xRange = {'min':47.3975, 'max':47.3980}
  #yRange = {'max':8.5455, 'min':8.545}
  #zRange = {'max':590, 'min':550}

  bounds = {}
  bounds['xMax'] = FAKE_GPS_ORIGIN_X + 0.0003
  bounds['xMin'] = FAKE_GPS_ORIGIN_X - 0.0003
  bounds['yMax'] = FAKE_GPS_ORIGIN_Y + 0.0003
  bounds['yMin'] = FAKE_GPS_ORIGIN_Y - 0.0003
  bounds['zMax'] = FAKE_GPS_ORIGIN_Z + 200
  bounds['zMin'] = FAKE_GPS_ORIGIN_Z + 150
  
  #~10000km per 90 degrees -> 111111.11m per degree
  print "[main] Drone Test Script Start!"

  #Need to handle manually setting each drone's ID
  #print "[main] no droneGPS created: simulation GPS used instead"

  drone = DroneModule.drone(bounds, DroneID)
  print "[main] Drone id %d created" % DroneID

  #TODO: Test aging exponent in sim
  print "[main] Starting loop!"
  
  #TODO: Add a blocking barrier here waiting for take off message
  drone.start()

  loop = 1
  while not rospy.is_shutdown():
    try:
      time.sleep(1)
      #print "[main] [%d] Iteration complete ----------" %(loop)
      #loop = loop + 1

    except KeyboardInterrupt:
      print "[main] Rospy shutdown detected. Stopping drone ROS modules!"
      drone.callback("end")
      print "[main] Drone modules stopped"

