#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32
from duckietown_msgs.msg import LanePose, Twist2DStamped, FSMState
import hw9_2.py as PID

class Follow:
    def __init__(self):
        kp = .45
        ki = 0.0001
        kd = 0.0001
        self.K = [kp, ki, kd]
        self.dt = 0.01
        self.tagPID = PID.PID(self.K, self.dt)
        
        self.tag = rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.control)
        self.mode = rospy.Subscriber("fsm_node/mode", FsmState, self.mode)
        self.pub = rospy.Publisher("lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)
        
        
    def control(self, msg):
        Vel = Twist2DStamped()
        
        if self.mode == "LANE_FOLLOWING":
            
            dErr = 0 - msg.d
            phiErr = 0 - msg.phi
        
            dSum = self.tagPID.run(dErr)
            phiSum = self.tagPID.run(phiErr)
            Err = dSum + phiSum
            
            trim = 0
            Vel.omega = trim - Err
            max_speed = 0.5
            Vel.v = max_speed - Err
            
            if Vel.v < 0:
                Vel.v = 0
        	    
            self.pub.publish(Vel)
        	
        elif self.mode == "NORMAL_JOYSTICK_CONTROL":
            Vel.v = 0
            Vel.omega = 0
            self.pub.publish(Vel)

        
        
    def mode(self, mode):
        self.state = mode.state
        
if __name__ == "__main__":
    rospy.init_node("lab4", anonymous=True)
    Follow()
    rospy.spin()
        
