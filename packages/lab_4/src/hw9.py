#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import hw9_2 as PID

class Establish:
    def __init__(self):
        rospy.set_param("controller_ready", "true")
        rospy.set_param("graph_ready", "true")
        kp = .45
        ki = 0.0001
        kd = 0.08
        self.K = [kp, ki, kd]
        self.dt = 0.01
        self.Distance_PID = PID.PID(self.K, self.dt)
        
        self.err = rospy.Subscriber("/error", Float32, self.control)
        self.pub = rospy.Publisher("/control_input", Float32, queue_size=10)
        self.i = 0
        
    def control(self, error):
                    
        rospy.logwarn(error.data)
        sumPID = self.Distance_PID.run(error.data)
        self.pub.publish(sumPID)
        #if self.i == 0:
            #rospy.sleep(2)
            #self.i = 1
        
if __name__ == "__main__":
    rospy.init_node("hw9", anonymous=True)
    Establish()
    rospy.spin()
        
