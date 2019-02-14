#!/usr/bin/env python

'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Sep 30 2018
                                      TODO

  File Name  : sim_utils.py
  Description: TODO
---*-----------------------------------------------------------------------*'''

PKG = 'drone'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import numpy
import time

import threading

from drone.msg import DroneComm

'''[radio_model]---------------------------------------------------------------
  Simulates radio congestion/utilization for drones
----------------------------------------------------------------------------'''
class radio_model:
  def __init__(self):
    self.links = {}
    self.cong_model = {}

  def add_link(self, d1, d2, freq):
    self.links[(d1, d2)] = [freq, 1.0 / freq]

  def del_link(self, d1, d2):
    if (d1, d2) in self.links:
      del self.links[(d1, d2)]

    if (d2, d1) in self.links:
      del self.links[(d2, d1)]
  
  def use_link(self, d1, d2):
    if (d1, d2) in self.links:
      pass

    if (d2, d1) in self.links:
      pass

    if (d1, d2) not in self.links and (d2, d1) not in self.links:
      print "[rad] Attempt to use invalid link [%d %d]" % (d1, d2)



    
    


