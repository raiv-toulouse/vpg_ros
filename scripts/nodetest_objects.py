#!/usr/bin/env python
# coding: utf-8

import rospy
from vpg_ros.srv import AddObjects,AddObjectsResponse,AddOneObject


def testObjet():
    # Test des objets
    rospy.wait_for_service('add_objects')
    try:
        print('node_objets')
        add_one_cube = rospy.ServiceProxy('add_one_cube', AddObjects)
        add_one_cube()
        add_objects = rospy.ServiceProxy('add_objects', AddObjects)
        add_objects(10)
        add_triangle = rospy.ServiceProxy('add_one_object', AddOneObject)
        add_triangle('triangle',[-0.5, 0.5, 0.01],[0, 0, 0],[0, 255, 0],)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    testObjet()

