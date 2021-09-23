#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled

class MysteryNode:
    def __init__(self):
        rospy.Subscriber("output2", UnitsLabelled, self.callback)
        self.pub_units = rospy.Publisher("conversion", UnitsLabelled, queue_size=10)
        self.pub_msg = UnitsLabelled()
        self.pub_msg.units = "feet"
        

    def callback(self, pub_msg):
        self.pub_msg.value = self.pub_msg.value * 3.28
        self.pub_units.publish(self.pub_msg)
        

if __name__ == '__main__':
    rospy.init_node('mystery_node')
    MysteryNode()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

