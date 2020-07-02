#!/usr/bin/env python
# coding: utf-8

import rospy
import numpy as np
from vpg_ros.cameraVREP import CameraVREP
from vpg_ros.objetsVREP import ObjetsVREP
from vpg_ros.robotVREP import RobotVREP


if __name__ == "__main__":
        rospy.init_node('node_environment_server')
        workspace_limits = np.asarray([[-0.724, -0.276], [-0.224, 0.224], [-0.0001, 0.4]])
        ipVREP = '127.0.0.1'
        myRobot = RobotVREP(workspace_limits,ipVREP)
        print("Ready RobotVREP")
        myObjects = ObjetsVREP(workspace_limits, '/home/phil/catkin_ws/src/vpg_ros/objects/blocks', 10)  # Se mettre dans le répertoire catkin_ws
        print("Ready ObjetsVREP")
        myCamera = CameraVREP()
        print("Ready CameraVREP")
        rospy.spin()

