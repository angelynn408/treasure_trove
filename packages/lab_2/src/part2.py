#!/usr/bin/env python3

from math import radians, sin, cos, sqrt
import numpy
import rospy
from duckietown_msgs.msg import WheelEncoderStamped, Pose2DStamped

class OdomNode:
    def __init__(self):
        self.pose = Pose2DStamped()
        self.R = rospy.Publisher("pose", Pose2DStamped, queue_size=10)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.s_l = 0
        self.s_r = 0
        self.delta_s_r = 0
        self.delta_s_l = 0
        self.delta_theta = 0
        radius = .065/2
        self.circumference = radius*2*numpy.pi
        self.rev_per_tick = 135
        self.dist_wheel_left = 0
        self.dist_wheel_right = 0  
        self.left_first = True
        self.right_first = True
        self.left_flag_new = False
        self.right_flag_new = False
        self.left_tick = rospy.Subscriber("left_wheel_encoder_node/tick", WheelEncoderStamped, self.Left_Wheel)
        self.right_tick = rospy.Subscriber("right_wheel_encoder_node/tick", WheelEncoderStamped, self.Right_Wheel)
       
                
    def Left_Wheel(self, msg):
        #number of revolutions
        revs_left = msg.data/self.rev_per_tick
        #calculation of distance based on above
        dist_left = self.circumference*revs_left
        self.delta_s_l = dist_left-self.dist_wheel_left
        self.dist_wheel_left = dist_left
        if self.left_first == True:
            self.left_first = False
        else:
            self.left_flag_new = True
        
    
    def Right_Wheel(self, msg):
        #number of revolutions
        revs_right = msg.data/self.rev_per_tick
        #calculation of distance based on above
        dist_right = self.circumference*revs_right
        self.delta_s_r = dist_right-self.dist_wheel_right
        self.dist_wheel_right = dist_right
        if self.right_first == True:
            self.right_first = False
        else:
            self.right_flag_new = True
    
    def callback_function(self):
        if self.right_flag_new == True and self.left_flag_new == True:  
            L = .05
            delta_s_r = self.delta_s_r
            delta_s_l = self.delta_s_l
            delta_s = (delta_s_r+delta_s_l)/2
            self.delta_theta = (delta_s_r-delta_s_l)/(2*L)
            delta_x = delta_s*cos(self.theta+self.delta_theta/2)
            delta_y = delta_s*sin(self.theta+self.delta_theta/2)
            self.x = self.x+delta_x
            self.y = self.y+delta_y
            self.theta = self.theta+self.delta_theta 
            self.pose.x = self.x
            self.pose.y = self.y
            self.pose.theta = self.theta                      
            self.R.publish(self.pose)
            self.right_flag_new = False
            self.left_flag_new = False      
                          
if __name__=='__main__':
    rospy.init_node ('odom_node', anonymous=True)
    O = OdomNode()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        O.callback_function()
        rate.sleep()
    
