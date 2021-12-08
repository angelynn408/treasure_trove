#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState

class ShapeNode:
    def __init__(self):
        self.pub = rospy.Publisher('lane_controller_node/car_cmd', Twist2DStamped, queue_size=10)
        self.sub = rospy.Subscriber('fsm_node/mode', FSMState, self.callback_function)
        self.flag = False
                        
    def callback_function(self, msg):      
        fwd = Twist2DStamped()
        if msg.state == "LANE_FOLLOWING" and self.flag == False:
            self.flag = True
            i = 0
            while i < 4:
                time = rospy.get_time()
                while rospy.get_time() < time + 2.7:
                    fwd.v = 0.6
                    fwd.omega = -0.4             #7-Dec -0.8
                    self.pub.publish(fwd)
                    rospy.sleep(0.1)
                fwd.v = 0
                fwd.omega = 0
                self.pub.publish(fwd)
                time = rospy.get_time()
                while rospy.get_time() < time + 0.8:
                    fwd.v = 0
                    fwd.omega = 1.2
                    self.pub.publish(fwd)
                    rospy.sleep(0.1)
                fwd.v = 0
                fwd.omega = 0                
                self.pub.publish(fwd)
                i = i + 1          	
        elif msg.state == "NORMAL_JOYSTICK_CONTROL":
            fwd.v = 0
            fwd.omega = 0
            self.flag = False
         
        self.pub.publish(fwd)
         
                 
if __name__=='__main__':
    rospy.init_node('line', anonymous=True)
    ShapeNode()
    rospy.spin()
