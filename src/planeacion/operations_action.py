#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float64MultiArray
from hola_tortuga.msg import Finished
from rospy import ROSException

def active_cb():
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    rospy.loginfo("Current location: "+str(feedback))

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")
    
if __name__ == "__main__":
        
    rospy.init_node('send_goal')

    navclient = actionlib.SimpleActionClient('/robot_1/motion_planner_action',MoveBaseAction)
    navclient.wait_for_server()

    # Example of navigation goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.position.y = 1.0
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.662
    goal.target_pose.pose.orientation.w = 0.750

    navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
    finished = navclient.wait_for_result()

    if not finished:
        rospy.logerr("Action server not available!")
    else:
        rospy.loginfo ( navclient.get_result())


