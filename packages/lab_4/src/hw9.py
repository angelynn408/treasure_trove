#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Establish:
    def __init__(self):
        rospy.set_param("controller_ready", "true")
        rospy.set_param("graph_ready", "true")
        kp = 1
        ki = 0
        kd = 0
        self.K = [kp, ki, kd]
        self.dt = 0.01
        self.Distance_PID = PID.PID(self.K, self.dt)
        
        self.err = rospy.Subscriber("/error", Float32, self.control)
        self.pub = rospy.Publisher("/control_input", Float32, queue_size = 10)
        
    def control(self, error):
        rospy.login(error.data)
        sumPID = self.Distance_PID.run(error.data)
        self.pub.publish(sumPID)
        
if __name__ == "__main__":
    rospy.init_mode("establish", anonymous=True)
    Establish()
    rospy.spin()
        
