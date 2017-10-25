#!/usr/bin/env python

import roslib
import math
import rospy
import numpy
from sensor_msgs.msg import LaserScan

class noisySensor:
  def __init__(self):
    rospy.init_node('noisy_base_scan')
    self.publisher=rospy.Publisher('/noisy_base_scan',LaserScan,queue_size=2)
    self.subscriber=rospy.Subscriber('/base_scan',LaserScan,self.scanReceived)
    try:
      self.mean=rospy.get_param('sensor_mean',0)
      self.sd=rospy.get_param('sensor_sd',0.1)
    except (KeyError):
      self.sd=0.1
      self.mean=0
  
  def scanReceived(self,laserScan):
    nlsr=map(lambda x:x+numpy.random.normal(self.mean,self.sd),laserScan.ranges)
    for i in range(0,len(laserScan.ranges)):
      laserScan.ranges=nlsr
    self.publisher.publish(laserScan)

ns=noisySensor()
rospy.spin()
    
