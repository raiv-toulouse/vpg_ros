#!/usr/bin/env python
# coding: utf-8

import rospy
from vpg_ros.srv import GripperCmd,CoordAction

def cmd(ouvrir):
    rospy.wait_for_service('cmd_gripper')
    try:
        cmdGripper = rospy.ServiceProxy('cmd_gripper', GripperCmd)
        resp1 = cmdGripper(ouvrir)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def grasp():
    rospy.wait_for_service('robot_grasp')
    try:
        cmdGrasp = rospy.ServiceProxy('robot_grasp', CoordAction)
        resp1 = cmdGrasp([-0.65,0.1,0.1],0.1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def push():
    rospy.wait_for_service('robot_push')
    try:
        cmdPush = rospy.ServiceProxy('robot_push', CoordAction)
        resp1 = cmdPush([-0.4,0.1,0.1],0.1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    cmd(True)
    rospy.sleep(2)
    cmd(False)
    rospy.sleep(2)
    grasp()
    push()
    print("Fin")
