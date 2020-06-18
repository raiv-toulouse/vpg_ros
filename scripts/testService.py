#!/usr/bin/env python


import numpy as np
from vpg_ros.srv import TestService,TestServiceResponse
import rospy


def envoyer(req):
    #t = [1.1, 2.2, 3.3, 4.4, 5.5]
    t = np.array([[1.1, 2.2], [3.3, 4.4]])
    print(type(t))
    return TestServiceResponse(tuple(t.reshape(1,-1)[0]),2,2)


rospy.init_node('testService_server')
s = rospy.Service('test_service', TestService, envoyer)
rospy.spin()