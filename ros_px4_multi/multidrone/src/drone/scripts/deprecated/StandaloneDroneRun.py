#!/usr/bin/env python

PKG = 'drone'

import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import math
import numpy
import random
import time
import json
import DroneModule;
import FakeDroneSensor;
import FakeDroneGPS;
import CommRangeDetector;

if __name__ == "__main__":
	# Desired input: Drone exploration area + drone mapping resolution
	# xMin, xMax, yMin, yMax, zMin, zMax, xRes, yRes, zRes, 
	#if len(sys.argv) != 16:
	#    print "Need drone input: xMin, xMax, yMin, yMax, zMin, zMax, xRes, yRes, zRes, xStart1, yStart1, zStart1, xStart2, yStart2, zStart2,. Did not detect all/only listed input, stopping...."

        rospy.init_node("dronetest", anonymous = True)

	xRange = {'min':-35.36338, 'max':-35.36320}
	yRange = {'max':149.16535, 'min':149.16517}
	zRange = {'max':650, 'min':630}

	# Desired grid resolution, number of drones, and communication radius (drone-to-drone range)
	resolutions = {'x':40,'y':40,'z':40}

	xStart1 = -35.36326
	yStart1 = 149.16523
	zStart1 = 635

	xStart2 = -35.36332
	yStart2 = 149.16523
	zStart2 = 635

	#~10000km per 90 degrees -> 111111.11m per degree
	print "Drone Test Script Start!"

	#Need to handle manually setting each drone's ID
	droneGPS1 = FakeDroneGPS.FakeDroneGPS(xStart1, yStart1, zStart1, 1)
	droneGPS2 = FakeDroneGPS.FakeDroneGPS(xStart2, yStart2, zStart2, 2)
	print "droneGPS created"

	#Fake gas sensor module data
	#randomFlag: 0 = seeded random values, 1 = non-seeded random values
	randomFlag = 0
	gasSensorModule1 = FakeDroneSensor.FakeDroneSensor(xRange, yRange, zRange, resolutions, randomFlag, 1, 1, 3)
	gasSensorModule2 = FakeDroneSensor.FakeDroneSensor(xRange, yRange, zRange, resolutions, randomFlag, 2, 1, 3)
	print "gasSensorModule created"

	commDetector = CommRangeDetector.CommRangeDetector(2,5)
	print "commDetection created"

	drone1 = DroneModule.Drone(xRange, yRange, zRange, resolutions, 1)
	drone2 = DroneModule.Drone(xRange, yRange, zRange, resolutions, 2)
	print "Drones created"
        droneGPS1.PublishGPS()
        droneGPS2.PublishGPS()
        drone1.PublishObj()
        drone2.PublishObj()


	#TODO: Test aging exponent in sim
	print "Starting loop!"

        loop = 1
	while (True):
            drone1.PublishObj()
            drone2.PublishObj()
	    droneGPS1.FakeMove()
	    droneGPS2.FakeMove()
	    commDetector.ProcessDroneSetGPS()
	    gasSensorModule1.PublishGasSensorValue()
	    gasSensorModule2.PublishGasSensorValue()
	    drone1.UpdateStatus()
	    drone2.UpdateStatus()
            print "Completed loop %d!" %(loop)
            loop = loop + 1
	    time.sleep(1.0)





