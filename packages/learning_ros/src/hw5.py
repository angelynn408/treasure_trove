#!/usr/bin/env python3

from math import radians, sin, cos, sqrt
import numpy
import rospy
from duckietown_msgs.msg import Vector2D

class TransformNode:
    def __init__(self):
          self.R = rospy.Publisher("robot", Vector2D, queue_size=10)
          self.W = rospy.Publisher("world", Vector2D, queue_size=10)
          R_Px = 2            #X-position of Robot
          R_Py = 7            #Y-position of Robot
          R_O = radians(135)  #Orientation
          R_Px_s = -2
          R_Py_s = 0
          self.R_T_s = numpy.matrix([[-1,0,-2],
                                [0,-1,0],
                                [0,0,1]])
          self.W_T_R = numpy.matrix([[-1/sqrt(2),-1/sqrt(2),2],
                                [1/sqrt(2),-1/sqrt(2),7],
                                [0,0,1]])
          self.W_T_S = self.W_T_R*self.R_T_s
          rospy.Subscriber("transform", Vector2D, self.callback_function)
          
    def callback_function(self, msg):
          s_p = numpy.matrix([[msg.x],[msg.y],[1]])
          robot_p = self.R_T_s*s_p
          world_p = self.W_T_R*robot_p
                           
          self.R.publish(robot_p[0,0],robot_p[1,0]) 
          self.W.publish(world_p[0,0],world_p[1,0])
                 
if __name__=='__main__':
    rospy.init_node ('transform_node')
    TransformNode()
    
    rospy.spin()
          
