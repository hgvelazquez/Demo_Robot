#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

global JOINT_POSITIONS
JOINT_POSITIONS = []

NUM_INTERPOLATIONS = 500

global TARGET_JOINT_POSITIONS
TARGET_JOINT_POSITIONS = []

global JOINT_VELOCITIES
JOINT_VELOCITIES = []

global POSITION_STEPS
POSITION_STEPS = []

global JOINT_NAMES
JOINT_NAMES = []

def jointStatesCallback(msg):
    print(len(msg.name))
    print(len(msg.position))
    if (len(msg.name) == 7):
        global JOINT_NAMES
        JOINT_NAMES = msg.name
        print
        print(msg)
        print

        temp_list = msg.position

        global JOINT_POSITIONS
        JOINT_POSITIONS = msg.position

        print
        print(JOINT_POSITIONS)
        print

        global JOINT_VELOCITIES
        JOINT_VELOCITIES = msg.velocity

        global JOINT_EFFORTS
        JOINT_EFFORTS = msg.effort

def jointStatePublisher():
    # Create ROS publisher
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

    global JOINT_POSITIONS 
    global JOINT_VELOCITIES

    j = 0
    while j < 2:
        robot_arm_joint_state.header.stamp = rospy.Time.now()
        new = []
        print
        if len(JOINT_POSITIONS) == 7 :
            for i in range(len(JOINT_POSITIONS)):
                if i >= 2:
                    new.append(JOINT_POSITIONS[i] + .01)
                else:
                    new.append(JOINT_POSITIONS[i])
            print
            print(new)
            print

            JOINT_POSITIONS = new
            robot_arm_joint_state.position = JOINT_POSITIONS

            data_publisher.publish(robot_arm_joint_state)
            j += 1
            # print
            # print(robot_arm_joint_state)
            # print
            rate.sleep()

    vel = [0,0,0,0,0,0,0]
    vel[2] = 0.5
    robot_arm_joint_state.velocity = vel
    data_publisher.publish(robot_arm_joint_state)
    while not rospy.is_shutdown():
        if not(JOINT_VELOCITIES is None):
            if(JOINT_VELOCITIES[2] < -3.13):
                JOINT_VELOCITIES[2]
                n_vel = list(JOINT_VELOCITIES)
                n_vel[2] = 0.1
                data_publisher.publish(robot_arm_joint_state)
            elif(JOINT_VELOCITIES[2] > 3.13):
                JOINT_VELOCITIES[2]
                n_vel = list(JOINT_VELOCITIES)
                n_vel[2] = -0.1
                data_publisher.publish(robot_arm_joint_state)





if __name__ == "__main__":
    try:
        # Create joint states' subscriber
        data_subscriber = rospy.Subscriber('joint_states', JointState, jointStatesCallback)
        jointStatePublisher()
    except rospy.ROSInterruptException:
        pass