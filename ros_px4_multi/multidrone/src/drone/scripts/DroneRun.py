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
from std_msgs.msg import String
 
'''[GLOBAL VARS]------------------------------------------------------------'''
GPS_ORIGIN_X = 47.39774
GPS_ORIGIN_Y = 8.54561
GPS_ORIGIN_Z = 408.0


'''[Message Handler]-----------------------------------------------------------
  Listen to topic uav<id>/ready, return true to isReady only after a message 
  "Go" received
----------------------------------------------------------------------------'''
class TakeOffMsgSubscriber:
  def __init__(self,id):
    self.sub_name = "uav%d/ready" % id
    self.ready = False
    self.pub = rospy.Subscriber(self.sub_name, String, self.ready_msg_callback)
    
  def ready_msg_callback(self, msg):
    if msg.data == "GO":
      self.ready = True
  
  def isReady(self):
    return self.ready


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

  gps = DroneModule.pos_gps_subscriber("uav%d/mavros/global_position/global" % DroneID)
  
  #Works, but is way out there
  #xRange = {'min':47.3975, 'max':47.3980}
  #yRange = {'max':8.5455, 'min':8.545}
  #zRange = {'max':590, 'min':550}
  
  # Get GPS
  print "Waiting for GPS signal coming in"
  gps_pos = gps.get_pos()
  GPS_ORIGIN_X = -1000
  while(GPS_ORIGIN_X< -180.0 or GPS_ORIGIN_X>180.0):
    GPS_ORIGIN_X = gps_pos['longitude']
    GPS_ORIGIN_Y = gps_pos['latitude']
    GPS_ORIGIN_Z = gps_pos['altitude']
    time.sleep(1)
  print "GPS Signal Received, current position (%f,%f,%f)" % (GPS_ORIGIN_X,GPS_ORIGIN_Y,GPS_ORIGIN_Z)

  bounds = {}
  bounds['xMax'] = GPS_ORIGIN_X + 0.0003
  bounds['xMin'] = GPS_ORIGIN_X - 0.0003
  bounds['yMax'] = GPS_ORIGIN_Y + 0.0003
  bounds['yMin'] = GPS_ORIGIN_Y - 0.0003
  bounds['zMax'] = GPS_ORIGIN_Z + 2
  bounds['zMin'] = GPS_ORIGIN_Z - 0.0003
  
  #~10000km per 90 degrees -> 111111.11m per degree
  print "[main] Drone Test Script Start!"

  drone = DroneModule.drone(bounds, DroneID)
  print "[main] Drone id %d created" % DroneID

  # #a blocking barrier here waiting for take off message
  # takeOffListener = TakeOffMsgSubscriber(DroneID)  
  # print "[main] Wait for take-off mesaage"
  # while not takeOffListener.isReady():
  #   time.sleep(1)

  # print "[main] Message received, Starting loop!"
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

