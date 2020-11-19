#!/usr/bin/env python3

import rospy
from vpg_ros.cameraVREP import CameraVREP


if __name__ == "__main__":
    rospy.init_node('node_cam_server')
    myCamera = CameraVREP()
    print("Ready CameraVREP")
    rospy.spin()
