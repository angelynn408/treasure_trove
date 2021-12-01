#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState

class ShapeNode:
    def __init__(self):
          self.pub = rospy.Publisher('lane_controller_node/car_cmd', Twist2DStamped, queue_size=10)
          self.sub = rospy.Subscriber('fsm_node/mode', FSMState, queue_size=10)
          self.flag = False
                        
    def callback_function(self, msg):      
          fwd = Twist2DStamped()
          if msg.state == "LANE_FOLLOWING" and self.flag == False:
          	self.flag = True
          	time = rospy.get_time()
          	while rospy.get_time() < time + 1.9:
          		fwd.v = 0.8
          		fwd.omega = -0.5
          		self.pub.publish(V)
          		rospy.sleep(0.2)
          	fwd.v = 0
          	fwd.omega = 0
          	self.pub.publish(V)          	
          elif msg.state == "JOYSTICK_CONTROL"
          	fwd.v = 0
          	fwd.omega = 0
         	self.flag = False
         
    	  self.pub.publish(V)
         
                 
if __name__=='__main__':
    rospy.init_node ('line', anonymous=true)
    ShapeNode()
    
    rospy.spin()
