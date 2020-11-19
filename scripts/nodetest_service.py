#!/usr/bin/env python

import rospy
from vpg_ros.srv import TestService,TestServiceResponse
import numpy as np

rospy.wait_for_service('test_service')
try:
    print('test service')
    test = rospy.ServiceProxy('test_service', TestService)
    resp = test()
    print(type(resp.tab_i[0]))
    print(resp.nbLignes)
    print(resp.nbColonnes)
    print(type(resp.tab_i))
    tab_i = np.asarray(resp.tab_i).reshape(resp.nbLignes,resp.nbColonnes,3)
    print(tab_i)
    tab_f = np.asarray(resp.tab_f).reshape(resp.nbLignes,resp.nbColonnes,3)
    print(tab_f)
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)