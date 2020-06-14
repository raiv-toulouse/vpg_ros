#!/usr/bin/env python

import rospy
from vpg_ros.srv import ColorDepthImages,ColorDepthImagesResponse

# Test de la camera
rospy.wait_for_service('get_color_depth_images')
try:
    print('node_test_environment')
    getImages = rospy.ServiceProxy('get_color_depth_images', ColorDepthImages)
    resp = getImages()
    print(resp.colorImage)
    print(resp.depthImage)
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

