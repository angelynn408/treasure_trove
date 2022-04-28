#!/usr/bin/env python3

import rospy
import numpy
from std_msgs.msg import Float32


class PID:
    def __init__(self, K, dt):
        self.kp = K[0]
        self.ki = K[1]
        self.kd = K[2]
        self.dt = dt
        self.e_integrate = 0
        self.e_prev = None
        self.out = 0
        
    def run(self,error):
        e = error
        self.e_integrate += e
        if self.e_prev == None:
            e_dot = 0.0
        else:
            e_dot = (e - self.e_prev) / self.dt
        P = self.kp * e
        I = self.ki = self.e_integrate
        D = self.kd = e_dot
        self.e_prev = e
        sumPID = P + I + D
        return(sumPID)
        
if __name__ == "__main__":
    rospy.init_node("PID", anonymous=True)
    rospy.spin()
