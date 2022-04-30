#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped, FSMState
import hw9_2 as PID

class Follow:
    def __init__(self):
        kp = .45
        ki = 0.0001
        kd = 0.0001
        self.K = [kp, ki, kd]
        self.dt = 0.01
        self.tagPID = PID.PID(self.K, self.dt)
        
        rospy.Subscriber("apriltag_detector_node/detections", AprilTagDetectionArray, self.control)
        rospy.Subscriber("fsm_node/mode", FSMState, self.mode)
        self.pub = rospy.Publisher("lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)
        
        
    def control(self, msg):
        Vel = Twist2DStamped()
        self.tag = msg
        
        if self.state == "LANE_FOLLOWING":
            if len(msg.detections) == 0:
                Vel.v = 0
                Vel.omega = 0
            else: 
                z = msg.detections[0].transform.translation.z
                x = msg.detections[1].transform.translation.x
                O = np.arctan(x/z)
        
                OSum = self.tagPID.run(O)
                vSum = self.tagPID.run(v)
        	
                if theta > 0.2 or theta < -0.2:
                    Vel.omega = OSum
                else:
                    Vel.omega = 0
        	    
                if z > 0.2:
                    Vel.v = vSum
                else:
                    Vel.v = 0
            rospy.logwarn("v = " + str(Vel.v))
            rospy.logwarn("omega = " + str(Vel.omega))    
            self.pub.publish(Vel)
        	
        elif self.state == "NORMAL_JOYSTICK_CONTROL":
            Vel.v = 0
            Vel.omega = 0
            self.pub.publish(Vel)

        
        
    def mode(self, msg):
        self.state = msg.state
        
if __name__ == "__main__":
    rospy.init_node("lab_4", anonymous=True)
    Follow()
    rospy.spin()
        
