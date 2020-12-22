#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float64MultiArray
from hola_tortuga.msg import Finished
from rospy import ROSException

global TARGETS
TARGETS = []

global FINISHED
FINISHED = 0
global S
S = ""

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

def finishedCallback(msg):
    global FINISHED
    FINISHED = msg.finished
    
def wait_to_arrive():
    # Esperamos que para cuando termine el contador
    # el robot esté frente a la pelota
    global FINISHED
    i = 0
    while(FINISHED == 0 or i < 1000000):
        #print(FINISHED) 
        #print(i)
        i += 1

if __name__ == "__main__":
    try:
        # Give node name
        rospy.init_node('planner')
        
        if (not rospy.has_param("~robot_name")):
            rospy.logerr("plan.py: No param named 'robot_name'")

        try:
            S = rospy.get_param('~robot_name')
            rospy.logerr("plan.py: Got param %s", S)
        except ROSException:
            rospy.logerr("plan.py: Failed to get param 'robot_name'")

        # Nos suscribimos para saber las posiciones de los objetivos
        targets_subscriber = rospy.Subscriber("/targets_marker", MarkerArray, targetsCallback)

        # Nos suscribimos para saber las posiciones de los objetivos
        finished_subscriber = rospy.Subscriber("demo_finished", Finished, finishedCallback)
        # Creamos un ROS publisher para mandar la posición del objetivo
        goal_publisher = rospy.Publisher("/"+S+'/move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.logerr("plan.py: publishing to %s", S+'/move_base_simple/goal')
        # Creamos un ROS publisher para notificar a la simulación el cambio 
        # en la posición de las pelotas
        grabbed_publisher = rospy.Publisher('/grabbed_ball', Float64MultiArray, queue_size=1)
        
        # Set operation frequency
        rate = rospy.Rate(10) # 10hz

        # Esperamos a obtener las posiciones de todas las pelotas
        while not rospy.is_shutdown() and len(TARGETS) < 1:
            print("waiting for targets")
            rate.sleep()

        # Para cada pelota ejecutamos las mismas acciones
        for i in range(len(TARGETS)):

            # TO DO
            # Que navegación ofresca un servicio
            # para poder reemplazar este while
            j = 0
            while(j < 20):
                rate.sleep()
                j += 1

            # Notificamos a la navegación la ubicación de la pelota
            nav_goal = PoseStamped()
            nav_goal.pose.position = TARGETS[i].pose.position
            goal_publisher.publish(nav_goal)
            print("go to\n\n"+str(nav_goal))

            # Esperamos que llegue al destino
            j = 0
            while(FINISHED == 0 or j < 20):
                if (rospy.is_shutdown()):
                    break
                print(FINISHED) 
                print(j)
                j += 1
                rate.sleep()

            # Movemos el brazo para que agarre la pelota
            move_arm_client(True)

            # Notificamos a la simulación que la pelota ya fue agarrada
            notification = Float64MultiArray()
            notification.data.append(i)
            grabbed_publisher.publish(notification)
            print(notification.data)
            print("\n grabbed \n")

            # Notificamos a la navegación la obicación a donde queremos
            # llevar la pelota
            nav_goal = PoseStamped()
            nav_goal.pose.position.x = 0
            nav_goal.pose.position.y = 0
            goal_publisher.publish(nav_goal)
            print("go to\n\n"+str(nav_goal))

            # Esperamos que llegue al destino
            j = 0
            while(FINISHED == 0 or j < 20):
                if (rospy.is_shutdown()):
                    break
                print(FINISHED) 
                print(j)
                j += 1
                rate.sleep()

            # Esperamos que llegue al destino
            #wait_to_arrive()

            # Movemos el brazo para que agarre la pelota
            move_arm_client(False) 

            # Notificamos a la simulación que la pelota ya fue soltada
            notification = Float64MultiArray()
            notification.data.append(i)
            notification.data.append(0)
            notification.data.append(0)
            grabbed_publisher.publish(notification)
            print(notification.data)
            print("\n not grabnbed \n")

            
            rate.sleep()
           

    except rospy.ROSInterruptException:
        pass
