#!/usr/bin/env python
# coding: utf-8

import rospy
from vpg_ros.srv import GripperCmd,CoordAction
from vpg_ros.srv import AddObjects,AddObjectsResponse,AddOneObject

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
        resp1 = cmdGrasp([-0.5,0.0,0.05],0.1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def push():
    rospy.wait_for_service('robot_push')
    try:
        cmdPush = rospy.ServiceProxy('robot_push', CoordAction)
        resp1 = cmdPush([-0.6,-0.25,0.01],0.1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    # Add a cube and a rectangle
    rospy.wait_for_service('add_objects')
    add_one_cube = rospy.ServiceProxy('add_one_cube', AddObjects)
    add_one_cube()
    add_triangle = rospy.ServiceProxy('add_one_object', AddOneObject)
    add_triangle('triangle',[-0.5, -0.25, 0.01],[0, 0, 0],[0, 255, 0],)
    # Now, the robot can act
    cmd(True) # Open the gripper
    rospy.sleep(2)
    cmd(False) # Close the gripper
    rospy.sleep(2)
    grasp()
    push()
    print("Fin")
