#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse

# Posisión de cada una de las articulaciones del robot
global JOINT_POSITIONS
JOINT_POSITIONS = []

# Nombres de cada una de las articulaciones del robot
global JOINT_NAMES
JOINT_NAMES = []

def jointStatesCallback(msg):
    # Solo nos interesa cuando recibimos información de todas las
    # articulaciones incluidas las el brazo
    if (len(msg.name) == 8):
        global JOINT_NAMES
        JOINT_NAMES = msg.name

        global JOINT_POSITIONS
        JOINT_POSITIONS = msg.position


def move_to_target(target):
    global JOINT_NAMES
    global JOINT_POSITIONS

    finished = False
    while not finished:
        robot_arm_joint_state.header.stamp = rospy.Time.now()
        new = []
        print(JOINT_NAMES)
        print(JOINT_POSITIONS)
        print("target\n")
        print(target)
        finished = True
        if len(JOINT_POSITIONS) == 8 :
            for i in range(len(JOINT_POSITIONS)):
                if i >= 3 :
                    if (abs(JOINT_POSITIONS[i] - target[i]) >= .01):
                        if(JOINT_POSITIONS[i] < target[i]):
                            new.append(JOINT_POSITIONS[i] + .01)
                            finished = finished and False
                        else:
                            new.append(JOINT_POSITIONS[i] - .01)
                            finished = finished and False
                    else:
                        new.append(JOINT_POSITIONS[i])
                        finished = finished and True
                else:
                    new.append(JOINT_POSITIONS[i])
            print
            print(new)
            print

            JOINT_POSITIONS = new
            robot_arm_joint_state.position = JOINT_POSITIONS

            data_publisher.publish(robot_arm_joint_state)
            #j += 1
            # print
            # print(robot_arm_joint_state)
            # print
            rate.sleep()
    print("\nTarget reached!\n")


def get_target_open_gripper():
    global JOINT_POSITIONS
    JOINT_OBJECTIVE = list(JOINT_POSITIONS)
    JOINT_OBJECTIVE[6] = 0.01*3
    JOINT_OBJECTIVE[7] = 0.01*3
    return JOINT_OBJECTIVE

def get_target_close_gripper():
    global JOINT_POSITIONS
    JOINT_OBJECTIVE = list(JOINT_POSITIONS)
    JOINT_OBJECTIVE[6] = 0.01
    JOINT_OBJECTIVE[7] = 0.01
    return JOINT_OBJECTIVE

def get_target_move_down():
    global JOINT_POSITIONS
    JOINT_OBJECTIVE = list(JOINT_POSITIONS)
    JOINT_OBJECTIVE[3] = JOINT_OBJECTIVE[3] + (0.01*70)
    JOINT_OBJECTIVE[4] = JOINT_OBJECTIVE[4] + (0.01*120)
    JOINT_OBJECTIVE[5] = JOINT_OBJECTIVE[5] + (0.01*110)
    return JOINT_OBJECTIVE

def get_target_move_up():
    global JOINT_POSITIONS
    global JOINT_POSITIONS
    JOINT_OBJECTIVE = list(JOINT_POSITIONS)
    JOINT_OBJECTIVE[3] = 0.0
    JOINT_OBJECTIVE[4] = 0.0
    JOINT_OBJECTIVE[5] = 0.0
    return JOINT_OBJECTIVE

def handle_move_arm(instruction):
    if(instruction.data):
        JOINT_TARGET = get_target_open_gripper()
        move_to_target(JOINT_TARGET)
        print("\nGarra abierta\n")
        JOINT_TARGET = get_target_move_down()
        move_to_target(JOINT_TARGET)
        print("\nBrazo abajo\n")
        JOINT_TARGET = get_target_close_gripper()
        move_to_target(JOINT_TARGET)
        print("\nGarra cerrada\n")
        JOINT_TARGET = get_target_move_up()
        move_to_target(JOINT_TARGET)
        print("\nBrazo arriba\n")
    else:
        JOINT_TARGET = get_target_move_down()
        move_to_target(JOINT_TARGET)
        print("\nBrazo abajo\n")
        JOINT_TARGET = get_target_open_gripper()
        move_to_target(JOINT_TARGET)
        print("\nGarra abierta\n")
        JOINT_TARGET = get_target_move_up()
        move_to_target(JOINT_TARGET)
        print("\nBrazo arriba\n")
        JOINT_TARGET = get_target_close_gripper()
        move_to_target(JOINT_TARGET)
        print("\nGarra cerrada\n")

    return SetBoolResponse(True,"Arm moved succesfully!\n")


def move_arm_server():
    s = rospy.Service('move_arm', SetBool, handle_move_arm)
    print("Ready to move the arm.")
    rospy.spin()


if __name__ == "__main__":
    try:
        # Nos suscribimos para saber que pasa con las articulaciones
        data_subscriber = rospy.Subscriber('joint_states', JointState, jointStatesCallback)
        # Creamos un ROS publisher para mandar las nuevas posiciones de las articulaciones
        data_publisher = rospy.Publisher('joint_states_interpolated', JointState, queue_size=10)
        # Give node name
        rospy.init_node('joint_state_interpolated_publisher')
        # Set operation frequency
        rate = rospy.Rate(10) # 10hz

        # Create a robot joint state message
        robot_arm_joint_state = JointState()
        robot_arm_joint_state.header = Header()
        robot_arm_joint_state.header.stamp = rospy.Time.now()
        robot_arm_joint_state.name = JOINT_NAMES
        robot_arm_joint_state.velocity = []
        robot_arm_joint_state.effort = []

        move_arm_server()
        

    except rospy.ROSInterruptException:
        pass