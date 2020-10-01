# coding: utf-8

import rospy
import vpg_ros.vrep as vrep
import os,time
import numpy as np
from vpg_ros.srv import AddObjects,AddObjectsResponse


class ObjetsVREP(object):
    def __init__(self,workspace_limits, obj_mesh_dir, num_obj):
        s = rospy.Service('add_objects', AddObjects, self.add_objects)
        s = rospy.Service('add_one_cube', AddObjects, self.add_one_cube)
        ipVREP = '127.0.0.1'
        self.sim_client = vrep.simxStart(ipVREP, 20001, True, True, 5000, 5)  # Connect to V-REP on port 19997
        if self.sim_client == -1:
            print('Failed to connect to simulation (V-REP remote API server). Exiting.')
            exit()
        else:
            print('Connected to simulation on port {}'.format(self.sim_client))
        # Define colors for object meshes (Tableau palette)
        self.color_space = np.asarray([[78.0, 121.0, 167.0],  # blue
                                       [89.0, 161.0, 79.0],  # green
                                       [156, 117, 95],  # brown
                                       [242, 142, 43],  # orange
                                       [237.0, 201.0, 72.0],  # yellow
                                       [186, 176, 172],  # gray
                                       [255.0, 87.0, 89.0],  # red
                                       [176, 122, 161],  # purple
                                       [118, 183, 178],  # cyan
                                       [255, 157, 167]]) / 255.0  # pink
        self.workspace_limits = workspace_limits
        # Read files in object mesh directory
        self.obj_mesh_dir = obj_mesh_dir
        self.num_obj = num_obj
        self.mesh_list = os.listdir(os.path.abspath(self.obj_mesh_dir))
        # Randomly choose objects to add to scene
        self.obj_mesh_ind = np.random.randint(0, len(self.mesh_list), size=self.num_obj)
        self.obj_mesh_color = self.color_space[np.asarray(range(self.num_obj)) % 10, :]


    def add_objects(self,req):
        # Add each object to robot workspace at x,y location and orientation (random or pre-loaded)
        for object_idx in range(len(self.obj_mesh_ind)):
            curr_mesh_file = os.path.join(self.obj_mesh_dir, self.mesh_list[self.obj_mesh_ind[object_idx]])
            curr_mesh_file = os.path.abspath(curr_mesh_file)
            curr_shape_name = 'shape_%02d' % object_idx
            drop_x = (self.workspace_limits[0][1] - self.workspace_limits[0][0] - 0.2) * np.random.random_sample() + self.workspace_limits[0][0] + 0.1
            drop_y = (self.workspace_limits[1][1] - self.workspace_limits[1][0] - 0.2) * np.random.random_sample() + self.workspace_limits[1][0] + 0.1
            object_position = [drop_x, drop_y, 0.15]
            object_orientation = [2*np.pi*np.random.random_sample(), 2*np.pi*np.random.random_sample(), 2*np.pi*np.random.random_sample()]
            object_color = [self.obj_mesh_color[object_idx][0], self.obj_mesh_color[object_idx][1], self.obj_mesh_color[object_idx][2]]
            ret_resp,ret_ints,ret_floats,ret_strings,ret_buffer = vrep.simxCallScriptFunction(self.sim_client, 'remoteApiCommandServer',vrep.sim_scripttype_childscript,'importShape',[0,0,255,0], object_position + object_orientation + object_color, [curr_mesh_file, curr_shape_name], bytearray(), vrep.simx_opmode_blocking)
            if ret_resp == 8:
                print('Failed to add new objects to simulation. Please restart.')
                exit()
            time.sleep(2)
        return AddObjectsResponse()

    def add_one_cube(self,req):
        """
        Add only one cube in the middle of the wrokspace for debug purpose.
        BE CAREFULL : you'll have to change the objects directory
        :param req:
        :return:
        """
        # Add only one cube in the (-0.5, 0, 0.01) coordinate
        curr_mesh_file = '/home/phil/catkin_ws/src/vpg_ros/objects/newblocks/cube.obj'  # a cube
        curr_mesh_file = os.path.abspath(curr_mesh_file)
        curr_shape_name = 'shape_00'
        object_position = [-0.5, 0, 0.01] # in the middle of the workspace
        object_orientation = [0, 0, 0]
        object_color = [255, 0, 0]  # red
        ret_resp,ret_ints,ret_floats,ret_strings,ret_buffer = vrep.simxCallScriptFunction(self.sim_client, 'remoteApiCommandServer',vrep.sim_scripttype_childscript,'importShape',[0,0,255,0], object_position + object_orientation + object_color, [curr_mesh_file, curr_shape_name], bytearray(), vrep.simx_opmode_blocking)
        time.sleep(2)
        return AddObjectsResponse()