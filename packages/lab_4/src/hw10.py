#!/usr/bin/env python3

import rospy
import actionlib
from example_service.srv import *
import example_action_server.msg

def ServiceClient(n):
    ServT1 = rospy.get_time()
    rospy.wait_for_service('calc_fibonacci')
    try:
        fib = rospy.ServiceProxy('calc_fibonacci', Fibonacci)
        nth = fib(n)
        return nth.sequence
    except rospy.ServiceException as e:
        print("Service call failure %s" %e)
    ServT2 = rospy.get_time()
    rospy.loginfo("Service time is " +str(ServT2-ServT1))

def ActionClient(n):
    client = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)
    client.wait_for_server()
    goal = example_action_server.msg.FibonacciGoal(order=n)
    
    ActT1 = rospy.get_time()
    client.send_goal(goal)
    ActT2 = rospy.get_time()
    client.wait_for_result()
    ActT3 = rospy.get_time()
    
    Send_Time = ActT2 - ActT1
    rospy.loginfo("Action" +str(n)+ "Send Time is " +str(Send_Time))
    Wait_Time = ActT3 - ActT2 
    rospy.loginfo("Action" +str(n)+ "Wait Time is " +str(Wait_Time))
    return client.get_result()
    
if __name__=="__main__":
    rospy.init_node('hw10')
    Serv3 = ServiceClient(3)
    rospy.loginfo("Service Order 3 is " +str(Serv3))
    Serv15= ServiceClient(15)
    rospy.loginfo("Service Order 15 is " +str(Serv15))
    Act3 = ActionClient(3)
    rospy.loginfo("Action order 3 is" +str(Act3))
    Act15 = ActionClient(15)
    rospy.loginfo("Action order 15 is" +str(Act15))
