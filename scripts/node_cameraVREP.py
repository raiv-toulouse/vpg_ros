#!/usr/bin/env python

from vpg_ros.cameraVREP import CameraVREP
import rospy

if __name__ == "__main__":
    myCamera = CameraVREP()
    print("Ready CameraVREP")
    rospy.spin()