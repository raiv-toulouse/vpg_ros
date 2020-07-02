#!/usr/bin/env python
# coding: utf-8

import rospy
from vpg_ros.srv import AddObjects,AddObjectsResponse


def testObjet():
    # Test des objets
    rospy.wait_for_service('add_objects')
    try:
        print('node_objets')
        addObjects = rospy.ServiceProxy('add_objects', AddObjects)
        addObjects()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    testObjet()

