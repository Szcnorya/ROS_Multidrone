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
from drone.msg import NavSatFix
from drone.msg import GasSensorData
from drone.msg import DroneComm
from sensor_msgs.msg import NavSatFix

# TODO MICHAEL: Ignore this file you don't need to use it

class TestGPSPublisher:
    def __init__(self, id):
        self.id = id
        self.moduleName = "iris_%d/mavros/state" %id
        self.pub = rospy.Publisher(self.moduleName, NavSatFix, queue_size=1)

    def publishGPS(self, longi, lat, height):
        msg = NavSatFix()
        msg.longitude = longi
        msg.latitude = lat
        msg.altitude = height
        #print "FakeDroneGPS %d: sending GPS (%f|%f|%f) on %s" %(self.id, msg.longitude, msg.latitude, msg.altitude, self.moduleName)
        self.pub.publish(msg)

class TestObjSubscriber:
    def __init__(self, id):
        self.id = id
        self.moduleName = "uav%d/Obj" %id
        self.xObj = -1000
        self.yObj = -1000
        self.zObj = -1000
        self.sub = None
    def setSubscribe(self):
        if self.sub == None:
            self.sub = rospy.Subscriber(self.moduleName, NavSatFix, self.WaypointReceive)
    def WaypointReceive(self,msg):
        self.yObj = msg.longitude
        self.xObj = msg.latitude
        self.zObj = msg.altitude
        #print "FakeDroneGPS %d: received objective (%f|%f|%f) on %s" %(self.id, msg.longitude, msg.latitude, msg.altitude, self.moduleName)
    def GetObj(self):
        return (self.xObj, self.yObj, self.zObj)

# Drone Class. Maintains ID, exploration flag, GPS location, time, objective GPS location, exploration grid bounds, 
# resolution of exploration grid, data aggregator object, exploration mapping object.
class FakeDroneGPS:
    # Constructor. Requires start GPS location, drone id
    def __init__(self, xStart, yStart, zStart, id):
        self.id = id
        self.x = xStart
        self.y = yStart
        self.z = zStart
        self.xObj = -1000
        self.yObj = -1000
        self.zObj = -1000

        self.ROSGPSLOC = TestGPSPublisher(id)
        self.ROSGPSOBJ = TestObjSubscriber(id)
        self.ROSGPSOBJ.setSubscribe()
        self.ROSGPSOBJ.setSubscribe()

    def GetLoc(self):
        return (self.x,self.y,self.z)
       
    # Update drone GPS location based on input measurement
    def FakeMove(self):
        self.ROSGPSOBJ.setSubscribe()
        (newX, newY, newZ) = self.ROSGPSOBJ.GetObj()
        self.xObj = newX
        self.yObj = newY
        self.zObj = newZ
        #print "FakeDroneGPS %d: Current location: (%f|%f|%f)" %(self.id, self.y, self.x,self.z)
        #print "FakeDroneGPS %d: Current objective: (%f|%f|%f)" %(self.id, self.yObj, self.xObj, self.zObj)
        self.ROSGPSLOC.publishGPS(self.y,self.x,self.z)

        if (self.xObj == -1000 or self.xObj == -1000 or self.xObj == -1000):
            rospy.loginfo("FakeDroneGPS %d: Objective not received: not moving, GPS = (%f|%f|%f)" %(self.id, self.y, self.x,self.z))
            self.ROSGPSLOC.publishGPS(self.y,self.x,self.z)
        else:
            xDist = (self.x - self.xObj)*111111.11
            yDist = (self.y - self.yObj)*111111.11
            zDist = (self.z - self.zObj)
            #print "xDist = ", xDist
            #print "yDist = ", yDist
            #print "zDist = ", zDist
            if (xDist > 0):
                #print "far from x"
                self.x = self.x - 0.5/111111.11
            else:
                self.x = self.x + 0.5/111111.11
            if (yDist > 0):
                #print "far from y"
                self.y = self.y - 0.5/111111.11
            else:
                self.y = self.y + 0.5/111111.11
            if (zDist > 0):
                #print "far from z"
                self.z = self.z - 0.5
            else:
                self.z = self.z + 0.5
            #print "Fake move for FakeDroneGPS %d: location is now (%f|%f|%f)" %(self.id, self.y, self.x,self.z)
            self.ROSGPSLOC.publishGPS(self.y,self.x,self.z)
    # Update drone GPS location based on input measurement
    def PublishGPS(self):
        self.ROSGPSLOC.publishGPS(self.y,self.x,self.z)









