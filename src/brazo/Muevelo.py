#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import SetBool

def move_arm_client(action):
    rospy.wait_for_service('move_arm')
    try:
        move_arm = rospy.ServiceProxy('move_arm', SetBool)
        resp1 = move_arm(action)
        return resp1.message
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [Bool]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        action = eval(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting to move robot up")
    print("The response from service is "+move_arm_client(action))