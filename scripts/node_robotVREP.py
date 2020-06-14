#!/usr/bin/env python

from vpg_ros.robotVREP import RobotVREP
import rospy
import numpy as np


if __name__ == "__main__":
    workspace_limits = np.asarray([[-0.724, -0.276], [-0.224, 0.224], [-0.0001, 0.4]])
    ipVREP = '127.0.0.1'
    myRobot = RobotVREP(workspace_limits,ipVREP)
    print("Ready RobotVREP")
    rospy.spin()