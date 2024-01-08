#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from ur5_dynamics.msg import Tracker

# Function to check the gripper status
def gripper_status(msg):
    if msg.data:
        return True

# Function to turn on the vacuum gripper
def gripper_on(gripper_number):
    rospy.wait_for_service(f'/ur5/vacuum_gripper{gripper_number}/on')
    try:
        turn_on = rospy.ServiceProxy(f'/ur5/vacuum_gripper{gripper_number}/on', Empty)
        resp = turn_on()
        return resp
    except rospy.ServiceException as e:
        print(f"Service call for gripper {gripper_number} failed: {e}")

# Function to turn off the vacuum gripper
def gripper_off(gripper_number):
    rospy.wait_for_service(f'/ur5/vacuum_gripper{gripper_number}/off')
    try:
        turn_off = rospy.ServiceProxy(f'/ur5/vacuum_gripper{gripper_number}/off', Empty)
        resp = turn_off()
        return resp
    except rospy.ServiceException as e:
        print(f"Service call for gripper {gripper_number} failed: {e}")

# Function to trigger all the grippers based on the tracker flag
def trigger_all_grippers(msg):
    gripper_trigger = msg.flag2
    if gripper_trigger:
        # Turn on all the vacuum grippers
        for i in range(8):
            gripper_on(i)
    else:
        # Turn off all the vacuum grippers
        for i in range(8):
            gripper_off(i)

# Initialize the ROS node
rospy.init_node("ur5_gripper", anonymous=False)

# Subscribe to the gripper status topic
gripper_status_sub = rospy.Subscriber('/ur5/vacuum_gripper/grasping', Bool, gripper_status, queue_size=1)

# Subscribe to the 'cxy1' topic to trigger all the grippers
cxy_sub = rospy.Subscriber('cxy1', Tracker, trigger_all_grippers, queue_size=1)

# Start the ROS main loop
rospy.spin()
