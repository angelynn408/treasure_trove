#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Talker:
    def __init__(self):
        self.pub = rospy.Publisher('input', Float32, queue_size=10)
    
    def talk(self):
        hello_str = variable
        rospy.loginfo(hello_str)
        self.pub.publish(hello_str)
if __name__ == '__main__':
    try:
        rospy.init_node('hw2', anonymous=True)
        t = Talker()
        rate = rospy.Rate(1)# 1hz
        variable = 1
        variable_2 = 0
        while not rospy.is_shutdown():
            t.talk()
            rate.sleep()
            fibinocci_math = (variable + variable_2)
            variable_2 = variable
            variable = fibinocci_math
    except rospy.ROSInterruptException:
        pass
        
    # spin()
    rospy.spin()
