#!/usr/bin/env python3

from math import radians, sin, cos, sqrt
import numpy
import rospy
from odometry_hw.msg import DistWheel, Pose2D

class OdomNode:
    def __init__(self):
          self.R = rospy.Publisher("/pose", Pose2D, queue_size=10)
          self.x = 0
          self.y = 0
          self.theta = 0
          rospy.Subscriber("/dist_wheel", DistWheel, self.callback_function)
          
    def callback_function(self, msg):
          L = .05            #baseline in meters
          delta_s_r = msg.dist_wheel_right
          delta_s_l = msg.dist_wheel_left
          delta_s = (delta_s_r+delta_s_l)/2
          delta_theta = (delta_s_r-delta_s_l)/(2*L)
          delta_x = delta_s*cos(self.theta+delta_theta/2)
          delta_y = delta_s*sin(self.theta+delta_theta/2)
          self.x = self.x+delta_x
          self.y = self.y+delta_y
          self.theta = self.theta+delta_theta                       
          self.R.publish(self.x,self.y,self.theta) 
                           
if __name__=='__main__':
    rospy.init_node ('odom_node')
    OdomNode()
    
    rospy.spin()
