#!/usr/bin/env python

from vpg_ros.objetsVREP import ObjetsVREP
import rospy
import numpy as np

if __name__ == "__main__":
    workspace_limits = np.asarray([[-0.724, -0.276], [-0.224, 0.224], [-0.0001, 0.4]])
    myObjects = ObjetsVREP(workspace_limits, 'src/vpg_ros/objects/blocks', 10)
    print("Ready ObjetsVREP")
    rospy.spin()