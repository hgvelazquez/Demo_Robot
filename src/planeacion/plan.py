#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float64MultiArray

global TARGETS
TARGETS = []

# Metodo para crear un cliente para llamar
# al servivio que mueve el brazo robot
def move_arm_client(action):
    rospy.wait_for_service('move_arm')
    try:
        move_arm = rospy.ServiceProxy('move_arm', SetBool)
        resp1 = move_arm(action)
        return resp1.message
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Metodo para obtener las posiciones de las pelotas en la simulación
def targetsCallback(msg):
    global TARGETS
    # Solo nos interesa cuando recibimos información cuando
    # todas las posiciones de todas las pelotas son publicadas
    if (len(msg.markers) == 4):
        TARGETS = msg.markers

if __name__ == "__main__":
    try:
        # Nos suscribimos para saber las posiciones de los objetivos
        targets_subscriber = rospy.Subscriber("targets_marker", MarkerArray, targetsCallback)
        # Creamos un ROS publisher para mandar la posición del objetivo
        goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        # Creamos un ROS publisher para notificar a la simulación el cambio 
        # en la posición de las pelotas
        grabbed_publisher = rospy.Publisher('/grabbed_ball', Float64MultiArray, queue_size=1)
        # Give node name
        rospy.init_node('planner')
        # Set operation frequency
        rate = rospy.Rate(10) # 10hz

        # Esperamos a obtener las posiciones de todas las pelotas
        while len(TARGETS) < 1:
            print("waiting for targets")

        # Para cada pelota ejecutamos las mismas acciones
        for i in range(len(TARGETS)):
            # Notificamos a la navegación la obicación de la pelota
            nav_goal = PoseStamped()
            nav_goal.pose.position = TARGETS[i].pose.position
            goal_publisher.publish(nav_goal)
            print("go to"+str(nav_goal))

            # Esperamos que para cuando termine el contador
            # el robot esté frente a la pelota
            i = 0
            while(i < 100000000):
                i += 1
            # Movemos el brazo para que agarre la pelota
            move_arm_client(True)

            # Notificamos a la simulación que la pelota ya fue agarrada
            notification = Float64MultiArray()
            notification.data
            grabbed_publisher.publish()
            print("repeat")

    except rospy.ROSInterruptException:
        pass