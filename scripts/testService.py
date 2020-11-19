#!/usr/bin/env python


import numpy as np
from vpg_ros.srv import TestService,TestServiceResponse
import rospy


def envoyer(req):
    #t_float = [1.1, 2.2, 3.3, 4.4, 5.5]
    t_float = np.array([[[1.1, 1.2, 1.3], [2.1, 2.2, 2.3], [3.1, 3.2, 3.3], [4.1, 4.2, 4.3]],
                  [[21.1, 21.2, 21.3], [22.1, 22.2, 22.3], [23.1, 23.2, 23.3], [24.1, 24.2, 24.3]]
                  ], dtype=np.float64)
    t_int = np.array([[[1, 1, 1], [2, 2, 2], [3, 3, 3], [4, 4, 4]],
                  [[21, 21, 21], [22, 22, 22], [23, 23, 23], [24, 24, 24]]
                  ], dtype=np.uint8)
    print(t_int.shape)
    print(t_int)
    return TestServiceResponse(tuple(t_int.reshape(1,-1)[0]),tuple(t_float.reshape(1,-1)[0]),2,4)


rospy.init_node('testService_server')
s = rospy.Service('test_service', TestService, envoyer)
rospy.spin()