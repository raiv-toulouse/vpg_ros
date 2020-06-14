#!/usr/bin/env python

from vpg_ros.cameraVREP import CameraVREP
import rospy
from vpg_ros.vrep import *

if __name__ == "__main__":
    ipVREP = '127.0.0.1'
    sim_client = simxStart(ipVREP, 19997, True, True, 5000, 5)  # Connect to V-REP on port 19997
    myCamera = CameraVREP(sim_client)
    print("Ready CameraVREP")
    rospy.spin()