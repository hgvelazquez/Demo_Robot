#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from nav_msgs.msg import Odometry

def showOdometry1(msg):
    print("group1 Odometry")
    print msg.pose.pose.position

def showOdometry2(msg):
    print("group2 Odometry")
    print msg.pose.pose.position

if __name__ == "__main__":
    rospy.init_node('showOdometry', anonymous=True) #make node 
    rospy.Subscriber('robot_1/odom',Odometry,showOdometry1)
    rospy.Subscriber('robot_2/odom',Odometry,showOdometry2)
    rospy.spin()

