#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import time
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float64MultiArray
from hola_tortuga.msg import Finished
from rospy import ROSException


global RESOLUTION
RESOLUTION = 0.2   # metros/cuadro
global FINISHED_RB1
FINISHED_RB1 = False

global FINISHED_RB2
FINISHED_RB2 = False

global FINISHED_RB3
FINISHED_RB3 = False

global FINISHED_RB4
FINISHED_RB4 = False

global FINISHED_RB5
FINISHED_RB5 = False

global FINISHED_RB6
FINISHED_RB6 = False

# Metodo para crear un cliente para llamar
# al servivio que mueve el brazo robot
def move_arm_client(action):
    rospy.wait_for_service('move_arm')
    try:
        move_arm = rospy.ServiceProxy('move_arm', SetBool)
        resp1 = move_arm(action)
        print("terminó de mover el brazo")
        return resp1.message
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Metodo para obtener las posiciones de las pelotas en la simulación
def targetsCallback(msg):
    rospy.logerr("plan.py: Targets recieved")
    global TARGETS
    # Solo nos interesa cuando recibimos información cuando
    # todas las posiciones de todas las pelotas son publicadas
    if (len(msg.markers) == 4):
        TARGETS = msg.markers

def finishedCallback_rb1(msg):
    global FINISHED_RB1
    FINISHED_RB1 = msg.finished

def finishedCallback_rb2(msg):
    global FINISHED_RB2
    FINISHED_RB2 = msg.finished

def finishedCallback_rb3(msg):
    global FINISHED_RB3
    FINISHED_RB3 = msg.finished

def finishedCallback_rb4(msg):
    global FINISHED_RB4
    FINISHED_RB4 = msg.finished

def finishedCallback_rb5(msg):
    global FINISHED_RB5
    FINISHED_RB5 = msg.finished

def finishedCallback_rb6(msg):
    global FINISHED_RB6
    FINISHED_RB6 = msg.finished

def move_robot(goal_publisher,x,y):
    # Notificamos a la navegación la ubicación objetivo
    nav_goal = PoseStamped()
    nav_goal.pose.position.x = RESOLUTION*x
    nav_goal.pose.position.y = RESOLUTION*y
    goal_publisher.publish(nav_goal)
    time.sleep(.2)
    print("go to\n\n"+str(nav_goal))

    # # Esperamos que llegue al destino
    # j = 0
    
    # while(finished == 0 or j < 20):
    #     if (rospy.is_shutdown()):
    #         break
    #     print(finished) 
    #     print(j)
    #     j += 1
    #     rate.sleep()

if __name__ == "__main__":
    try:
        # Give node name
        rospy.init_node('operations')

        # Topicos necesarios para mover al robot_1
        # Nos suscribimos al topico que informa cuando ya se llegó al objetivo
        finished_subscriber_rb1 = rospy.Subscriber("/group1/motion/demo_finished", Finished, finishedCallback_rb1)
        # Creamos un ROS publisher para mandar la posición del objetivo
        goal_publisher_rb1 = rospy.Publisher("/robot_1/move_base_simple/goal", PoseStamped, queue_size=1)

        # Topicos necesarios para mover al robot_2
        # Nos suscribimos al topico que informa cuando ya se llegó al objetivo
        finished_subscriber_rb2 = rospy.Subscriber("/group2/motion/demo_finished", Finished, finishedCallback_rb2)
        # Creamos un ROS publisher para mandar la posición del objetivo
        goal_publisher_rb2 = rospy.Publisher("/robot_2/move_base_simple/goal", PoseStamped, queue_size=1)

        # Topicos necesarios para mover al robot_3
        # Nos suscribimos al topico que informa cuando ya se llegó al objetivo
        finished_subscriber_rb3 = rospy.Subscriber("/group3/motion/demo_finished", Finished, finishedCallback_rb3)
        # Creamos un ROS publisher para mandar la posición del objetivo
        goal_publisher_rb3 = rospy.Publisher("/robot_3/move_base_simple/goal", PoseStamped, queue_size=1)

        # Topicos necesarios para mover al robot_4
        # Nos suscribimos al topico que informa cuando ya se llegó al objetivo
        finished_subscriber_rb4 = rospy.Subscriber("/group4/motion/demo_finished", Finished, finishedCallback_rb4)
        # Creamos un ROS publisher para mandar la posición del objetivo
        goal_publisher_rb4 = rospy.Publisher("/robot_4/move_base_simple/goal", PoseStamped, queue_size=1)

        # Topicos necesarios para mover al robot_5
        # Nos suscribimos al topico que informa cuando ya se llegó al objetivo
        finished_subscriber_rb5 = rospy.Subscriber("/group5/motion/demo_finished", Finished, finishedCallback_rb5)
        # Creamos un ROS publisher para mandar la posición del objetivo
        goal_publisher_rb5 = rospy.Publisher("/robot_5/move_base_simple/goal", PoseStamped, queue_size=1)

        # Topicos necesarios para mover al robot_6
        # Nos suscribimos al topico que informa cuando ya se llegó al objetivo
        finished_subscriber_rb6 = rospy.Subscriber("/group6/motion/demo_finished", Finished, finishedCallback_rb6)
        # Creamos un ROS publisher para mandar la posición del objetivo
        goal_publisher_rb6 = rospy.Publisher("/robot_6/move_base_simple/goal", PoseStamped, queue_size=1)

        rospy.logerr("plan.py: publishing to /robot_1/move_base_simple/goal")
        rospy.logerr("plan.py: publishing to robot_2/move_base_simple/goal")
        # Creamos un ROS publisher para notificar a la simulación el cambio 
        # en la posición de las pelotas
        grabbed_publisher = rospy.Publisher('/grabbed_ball', Float64MultiArray, queue_size=1)
        
        # Set operation frequency
        rate = rospy.Rate(10) # 10hz

        time.sleep(1)
        
        global RESOLUTION

        move_robot(goal_publisher_rb1,2,1)
        move_robot(goal_publisher_rb2,0,2)
        # move_robot(goal_publisher_rb3,-5,-12)
        # move_robot(goal_publisher_rb4,10,-5)
        # move_robot(goal_publisher_rb5,-2,-5)
        # move_robot(goal_publisher_rb6,-4,5)
           

    except rospy.ROSInterruptException:
        pass
