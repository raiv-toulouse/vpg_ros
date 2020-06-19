#!/usr/bin/env python
# coding: utf-8

import rospy
from vpg_ros.srv import ColorDepthImages,ColorDepthImagesResponse
import numpy as np
from PIL import Image


def showImage(img,type):
    img = Image.fromarray(img, type)
    img.show()

def testCamera():
    # Test de la camera
    rospy.wait_for_service('get_color_depth_images')
    try:
        print('node_test_environment')
        getImages = rospy.ServiceProxy('get_color_depth_images', ColorDepthImages)
        resp = getImages()
        width = resp.width
        height = resp.height
        colorImage = np.asarray(resp.colorImage, dtype=np.uint8).reshape(width,height,3)
        depthImage = np.asarray(resp.depthImage, dtype=np.uint8).reshape(width,height)
        showImage(colorImage,'RGB')
        showImage(depthImage,'L')
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    testCamera()

