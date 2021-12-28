#!/usr/bin/env python3

from math import radians, sin, cos, sqrt
import numpy
import rospy
from duckietown_msgs.msg import WheelEncoderStamped, Pose2DStamped

class OdomNode:
    def __init__(self):
        self.pose = Pose2DStamped()
        self.R = rospy.Publisher("/pose", Pose2DStamped, queue_size=10)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.s_l = 0
        self.s_r = 0
        self.delta_s_r = 0
        self.delta_s_l = 0
        self.delta_s = 0
        self.delta_theta = 0
        radius = .065/2
        self.circumference = radius*2*numpy.pi
        self.rev_per_tick = 115
          
        self.left_tick = rospy.Subscriber("left_wheel_encoder_node/tick", WheelEncoderStamped, self.Left_Wheel)
        self.right_tick = rospy.Subscriber("right_wheel_encoder_node/tick", WheelEncoderStamped, self.Right_Wheel)
        rospy.Subscriber("/dist_wheel", DistWheel, self.callback_function)
                  
    def Left_Wheel(self, msg):
        #number of revolutions
        revs_left = msg.data/self.rev_per_tick
        #calculation of distance based on above
        dist_left = self.circumference*revs_left
        #delta distance new - s
        self.dist_wheel_left = dist_left-self.delta_s_l
        
    
    def Right_Wheel(self, msg):
        #number of revolutions
        revs_right = msg.data/self.rev_per_tick
        #calculation of distance based on above
        dist_right = self.circumference*revs_right
        self.dist_wheel_right = dist_right-self.delta_s_l
        
    
    def callback_function(self):
        L = .05
        delta_s_r = self.dist_wheel_right
        delta_s_l = self.dist_wheel_left
        delta_s = (delta_s_r+delta_s_l)/2
        delta_theta = (delta_s_r-delta_s_l)/(2*L)
        delta_x = delta_s*cos(self.theta+delta_theta/2)
        delta_y = delta_s*sin(self.theta+delta_theta/2)
        self.x = self.x+delta_x
        self.y = self.y+delta_y
        self.theta = self.theta+delta_theta 
        self.pose.x = self.x
        self.pose.y = self.y
        self.pose.theta = self.theta                      
        self.pub.publish(self.pose)
        
                           
if __name__=='__main__':
    rospy.init_node ('odom_node', anonymous=True)
    O = OdomNode()
    wait = rospy.Rate(10)
    while not rospy.is_shutdown():
        O.callback_function()
        wait.sleep()
    rospy.spin()
    
