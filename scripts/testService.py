#!/usr/bin/env python


import numpy as np
from vpg_ros.srv import TestService,TestServiceResponse
import rospy


def envoyer(req):
    #t = [1.1, 2.2, 3.3, 4.4, 5.5]
    t = np.array([[[1.1, 1.2, 1.3], [2.1, 2.2, 2.3], [3.1, 3.2, 3.3], [4.1, 4.2, 4.3]],
                  [[21.1, 21.2, 21.3], [22.1, 22.2, 22.3], [23.1, 23.2, 23.3], [24.1, 24.2, 24.3]]
                  ])
    print(t.shape)
    return TestServiceResponse(tuple(t.reshape(1,-1)[0]),2,4)


rospy.init_node('testService_server')
s = rospy.Service('test_service', TestService, envoyer)
rospy.spin()