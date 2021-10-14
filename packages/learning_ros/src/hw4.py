#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class MysteryNode:
    def __init__(self):
        rospy.Subscriber("output2", Float32, self.callback)
        self.pub_units = rospy.Publisher("conversion", Float32, queue_size=10)

    def callback(self, msg):
        self.pub_msg = msg
        unit=rospy.get_param("conversion")
        
        if unit == "feet":
          self.pub_msg.data = self.pub_msg.data * 3.28
          self.pub_units.publish(self.pub_msg)
          
        elif unit == "meters":
          self.pub_msg.data = self.pub_msg.data
          self.pub_units.publish(self.pub_msg)
          
        else:
           self.pub_msg.data = self.pub_msg.data * 0.588
           self.pub_units.publish(self.pub_msg)       
        
if __name__ == '__main__':
    rospy.init_node('mystery_node')
    MysteryNode()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

