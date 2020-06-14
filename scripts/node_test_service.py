#!/usr/bin/env python

import rospy
from vpg_ros.srv import TestService,TestServiceResponse

rospy.wait_for_service('test_service')
try:
    print('test service')
    test = rospy.ServiceProxy('test_service', TestService)
    resp = test()
    print(resp.tab1)
    print(type(resp.tab1))
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)