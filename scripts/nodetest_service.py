#!/usr/bin/env python

import rospy
from vpg_ros.srv import TestService,TestServiceResponse
import numpy as np

rospy.wait_for_service('test_service')
try:
    print('test service')
    test = rospy.ServiceProxy('test_service', TestService)
    resp = test()
    print(resp.tab1)
    print(resp.tab1.shape)
    print(resp.nbLignes)
    print(resp.nbColonnes)
    print(type(resp.tab1))
    a = np.asarray(resp.tab1).reshape(resp.nbLignes,resp.nbColonnes,3)
    print(a)
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)