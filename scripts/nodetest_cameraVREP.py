#!/usr/bin/env python
# coding: utf-8

import rospy
from vpg_ros.srv import ColorDepthImages
import numpy as np
from PIL import Image


def show_image(img, type):
    img = Image.fromarray(img, type)
    img.show()


def test_camera():
    # Test de la camera
    rospy.wait_for_service('get_color_depth_images')
    try:
        print('nodetest_cameraVREP')
        get_images = rospy.ServiceProxy('get_color_depth_images', ColorDepthImages)
        resp = get_images()
        width = resp.width
        height = resp.height
        color_image = np.asarray(resp.colorImage, dtype=np.uint8).reshape(width, height, 3)
        depth_image = np.asarray(resp.depthImage, dtype=np.float32).reshape(width, height)
        show_image(color_image, 'RGB')
        depth_image_gray = (depth_image * 255 / np.max(depth_image)).astype('uint8')
        show_image(depth_image_gray, 'L')
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    test_camera()
