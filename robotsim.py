import socket
import select
import struct
import time
import os
import numpy as np
from simulation import vrep
import utils

class RobotSim(object):
    def __init__(self, obj_mesh_dir, workspace_limits, is_testing, ip_vrep = '127.0.0.1'):
        self.workspace_limits = workspace_limits
        # Read files in object mesh directory
        self.obj_mesh_dir = obj_mesh_dir
        self.mesh_list = os.listdir(self.obj_mesh_dir)
        vrep.simxFinish(-1)  # Just in case, close all opened connections
        self.sim_client = vrep.simxStart(ip_vrep, 19997, True, True, 5000, 5)  # Connect to V-REP on port 19997
        if self.sim_client == -1:
            print('Failed to connect to simulation (V-REP remote API server). Exiting.')
            exit()
        else:
            print('Connected to simulation.')
            self.restart_sim()
        # Setup virtual camera in simulation
        self.setup_sim_camera()
        self.is_testing = is_testing

    def check_sim(self):
        # Check if simulation is stable by checking if gripper is within workspace
        sim_ret, gripper_position = vrep.simxGetObjectPosition(self.sim_client, self.RG2_tip_handle, -1, vrep.simx_opmode_blocking)
        sim_ok = gripper_position[0] > self.workspace_limits[0][0] - 0.1 and gripper_position[0] < self.workspace_limits[0][1] + 0.1 and gripper_position[1] > self.workspace_limits[1][0] - 0.1 and gripper_position[1] < self.workspace_limits[1][1] + 0.1 and gripper_position[2] > self.workspace_limits[2][0] and gripper_position[2] < self.workspace_limits[2][1]
        if not sim_ok:
            print('Simulation unstable. Restarting environment.')
            self.restart_sim()


    def restart_sim(self):
        sim_ret, self.UR5_target_handle = vrep.simxGetObjectHandle(self.sim_client,'UR5_target',vrep.simx_opmode_blocking)
        vrep.simxSetObjectPosition(self.sim_client, self.UR5_target_handle, -1, (-0.5,0,0.3), vrep.simx_opmode_blocking)
        vrep.simxStopSimulation(self.sim_client, vrep.simx_opmode_blocking)
        vrep.simxStartSimulation(self.sim_client, vrep.simx_opmode_blocking)
        time.sleep(1)
        sim_ret, self.RG2_tip_handle = vrep.simxGetObjectHandle(self.sim_client, 'UR5_tip', vrep.simx_opmode_blocking)
        sim_ret, gripper_position = vrep.simxGetObjectPosition(self.sim_client, self.RG2_tip_handle, -1, vrep.simx_opmode_blocking)
        while gripper_position[2] > 0.4: # V-REP bug requiring multiple starts and stops to restart
            vrep.simxStopSimulation(self.sim_client, vrep.simx_opmode_blocking)
            vrep.simxStartSimulation(self.sim_client, vrep.simx_opmode_blocking)
            time.sleep(1)
            sim_ret, gripper_position = vrep.simxGetObjectPosition(self.sim_client, self.RG2_tip_handle, -1, vrep.simx_opmode_blocking)


    def move_to(self, tool_position):

        # sim_ret, UR5_target_handle = vrep.simxGetObjectHandle(self.sim_client,'UR5_target',vrep.simx_opmode_blocking)
        sim_ret, UR5_target_position = vrep.simxGetObjectPosition(self.sim_client, self.UR5_target_handle, -1,
                                                                  vrep.simx_opmode_blocking)

        move_direction = np.asarray(
            [tool_position[0] - UR5_target_position[0], tool_position[1] - UR5_target_position[1],
             tool_position[2] - UR5_target_position[2]])
        move_magnitude = np.linalg.norm(move_direction)
        move_step = 0.02 * move_direction / move_magnitude
        num_move_steps =int(np.floor(move_magnitude/0.02))

        for step_iter in range(num_move_steps):
            vrep.simxSetObjectPosition(self.sim_client, self.UR5_target_handle, -1, (
            UR5_target_position[0] + move_step[0], UR5_target_position[1] + move_step[1],
            UR5_target_position[2] + move_step[2]), vrep.simx_opmode_blocking)
            sim_ret, UR5_target_position = vrep.simxGetObjectPosition(self.sim_client, self.UR5_target_handle, -1,
                                                                      vrep.simx_opmode_blocking)
        vrep.simxSetObjectPosition(self.sim_client, self.UR5_target_handle, -1,
                                   (tool_position[0], tool_position[1], tool_position[2]), vrep.simx_opmode_blocking)

    def open_gripper(self, async=False):

        gripper_motor_velocity = 0.5
        gripper_motor_force = 20
        sim_ret, RG2_gripper_handle = vrep.simxGetObjectHandle(self.sim_client, 'RG2_openCloseJoint',
                                                               vrep.simx_opmode_blocking)
        sim_ret, gripper_joint_position = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle,
                                                                    vrep.simx_opmode_blocking)
        vrep.simxSetJointForce(self.sim_client, RG2_gripper_handle, gripper_motor_force, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(self.sim_client, RG2_gripper_handle, gripper_motor_velocity,
                                        vrep.simx_opmode_blocking)
        while gripper_joint_position < 0.0536:  # Block until gripper is fully open
            sim_ret, gripper_joint_position = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle,
                                                                        vrep.simx_opmode_blocking)

    def close_gripper(self, async=False):

        gripper_motor_velocity = -0.5
        gripper_motor_force = 100
        sim_ret, RG2_gripper_handle = vrep.simxGetObjectHandle(self.sim_client, 'RG2_openCloseJoint',
                                                               vrep.simx_opmode_blocking)
        sim_ret, gripper_joint_position = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle,
                                                                    vrep.simx_opmode_blocking)
        vrep.simxSetJointForce(self.sim_client, RG2_gripper_handle, gripper_motor_force, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(self.sim_client, RG2_gripper_handle, gripper_motor_velocity,
                                        vrep.simx_opmode_blocking)
        gripper_fully_closed = False
        while gripper_joint_position > -0.047:  # Block until gripper is fully closed
            sim_ret, new_gripper_joint_position = vrep.simxGetJointPosition(self.sim_client, RG2_gripper_handle,
                                                                            vrep.simx_opmode_blocking)
            print(gripper_joint_position)
            if new_gripper_joint_position >= gripper_joint_position:
                return gripper_fully_closed
            gripper_joint_position = new_gripper_joint_position

    def get_camera_data(self):
        # Get color image from simulation
        sim_ret, resolution, raw_image = vrep.simxGetVisionSensorImage(self.sim_client, self.cam_handle, 0, vrep.simx_opmode_blocking)
        color_img = np.asarray(raw_image)
        color_img.shape = (resolution[1], resolution[0], 3)
        color_img = color_img.astype(np.float)/255
        color_img[color_img < 0] += 1
        color_img *= 255
        color_img = np.fliplr(color_img)
        color_img = color_img.astype(np.uint8)
        # Get depth image from simulation
        sim_ret, resolution, depth_buffer = vrep.simxGetVisionSensorDepthBuffer(self.sim_client, self.cam_handle, vrep.simx_opmode_blocking)
        depth_img = np.asarray(depth_buffer)
        depth_img.shape = (resolution[1], resolution[0])
        depth_img = np.fliplr(depth_img)
        zNear = 0.01
        zFar = 10
        depth_img = depth_img * (zFar - zNear) + zNear
        return color_img, depth_img

    def setup_sim_camera(self):

        # Get handle to camera
        sim_ret, self.cam_handle = vrep.simxGetObjectHandle(self.sim_client, 'Vision_sensor_persp', vrep.simx_opmode_blocking)

        # Get camera pose and intrinsics in simulation
        sim_ret, cam_position = vrep.simxGetObjectPosition(self.sim_client, self.cam_handle, -1, vrep.simx_opmode_blocking)
        sim_ret, cam_orientation = vrep.simxGetObjectOrientation(self.sim_client, self.cam_handle, -1, vrep.simx_opmode_blocking)
        cam_trans = np.eye(4,4)
        cam_trans[0:3,3] = np.asarray(cam_position)
        cam_orientation = [-cam_orientation[0], -cam_orientation[1], -cam_orientation[2]]
        cam_rotm = np.eye(4,4)
        cam_rotm[0:3,0:3] = np.linalg.inv(utils.euler2rotm(cam_orientation))
        self.cam_pose = np.dot(cam_trans, cam_rotm) # Compute rigid transformation representating camera pose
        self.cam_intrinsics = np.asarray([[618.62, 0, 320], [0, 618.62, 240], [0, 0, 1]])
        self.cam_depth_scale = 1

        # Get background image
        self.bg_color_img, self.bg_depth_img = self.get_camera_data()
        self.bg_depth_img = self.bg_depth_img * self.cam_depth_scale

    def grasp(self, position, heightmap_rotation_angle, workspace_limits):
        print('Executing: grasp at (%f, %f, %f)' % (position[0], position[1], position[2]))

        # Compute tool orientation from heightmap rotation angle
        tool_rotation_angle = (heightmap_rotation_angle % np.pi) - np.pi / 2

        # Avoid collision with floor
        position = np.asarray(position).copy()
        position[2] = max(position[2] - 0.04, workspace_limits[2][0] + 0.02)

        # Move gripper to location above grasp target
        grasp_location_margin = 0.15
        # sim_ret, UR5_target_handle = vrep.simxGetObjectHandle(self.sim_client,'UR5_target',vrep.simx_opmode_blocking)
        location_above_grasp_target = (position[0], position[1], position[2] + grasp_location_margin)

        # Compute gripper position and linear movement increments
        tool_position = location_above_grasp_target
        sim_ret, UR5_target_position = vrep.simxGetObjectPosition(self.sim_client, self.UR5_target_handle, -1,
                                                                  vrep.simx_opmode_blocking)
        move_direction = np.asarray(
            [tool_position[0] - UR5_target_position[0], tool_position[1] - UR5_target_position[1],
             tool_position[2] - UR5_target_position[2]])
        move_magnitude = np.linalg.norm(move_direction)
        move_step = 0.05 * move_direction / move_magnitude
        num_move_steps = int(np.floor(move_direction[0] / move_step[0]))

        # Compute gripper orientation and rotation increments
        sim_ret, gripper_orientation = vrep.simxGetObjectOrientation(self.sim_client, self.UR5_target_handle, -1,
                                                                     vrep.simx_opmode_blocking)
        rotation_step = 0.3 if (tool_rotation_angle - gripper_orientation[1] > 0) else -0.3
        num_rotation_steps = int(np.floor((tool_rotation_angle - gripper_orientation[1]) / rotation_step))

        # Simultaneously move and rotate gripper
        for step_iter in range(max(num_move_steps, num_rotation_steps)):
            vrep.simxSetObjectPosition(self.sim_client, self.UR5_target_handle, -1, (
            UR5_target_position[0] + move_step[0] * min(step_iter, num_move_steps),
            UR5_target_position[1] + move_step[1] * min(step_iter, num_move_steps),
            UR5_target_position[2] + move_step[2] * min(step_iter, num_move_steps)), vrep.simx_opmode_blocking)
            vrep.simxSetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, (
            np.pi / 2, gripper_orientation[1] + rotation_step * min(step_iter, num_rotation_steps), np.pi / 2),
                                          vrep.simx_opmode_blocking)
        vrep.simxSetObjectPosition(self.sim_client, self.UR5_target_handle, -1,
                                   (tool_position[0], tool_position[1], tool_position[2]),
                                   vrep.simx_opmode_blocking)
        vrep.simxSetObjectOrientation(self.sim_client, self.UR5_target_handle, -1,
                                      (np.pi / 2, tool_rotation_angle, np.pi / 2), vrep.simx_opmode_blocking)

        # Ensure gripper is open
        self.open_gripper()

        # Approach grasp target
        self.move_to(position, None)

        # Close gripper to grasp target
        gripper_full_closed = self.close_gripper()

        # Move gripper to location above grasp target
        self.move_to(location_above_grasp_target, None)

        # Check if grasp is successful
        gripper_full_closed = self.close_gripper()
        grasp_success = not gripper_full_closed

        # Move the grasped object elsewhere
        if grasp_success:
            object_positions = np.asarray(self.get_obj_positions())
            object_positions = object_positions[:, 2]
            grasped_object_ind = np.argmax(object_positions)
            grasped_object_handle = self.object_handles[grasped_object_ind]
            vrep.simxSetObjectPosition(self.sim_client, grasped_object_handle, -1,
                                       (-0.5, 0.5 + 0.05 * float(grasped_object_ind), 0.1),
                                       vrep.simx_opmode_blocking)


        return grasp_success

    def push(self, position, heightmap_rotation_angle, workspace_limits):
        print('Executing: push at (%f, %f, %f)' % (position[0], position[1], position[2]))

        # Compute tool orientation from heightmap rotation angle
        tool_rotation_angle = (heightmap_rotation_angle % np.pi) - np.pi / 2

        # Adjust pushing point to be on tip of finger
        position[2] = position[2] + 0.026

        # Compute pushing direction
        push_orientation = [1.0, 0.0]
        push_direction = np.asarray([push_orientation[0] * np.cos(heightmap_rotation_angle) - push_orientation[
            1] * np.sin(heightmap_rotation_angle),
                                     push_orientation[0] * np.sin(heightmap_rotation_angle) + push_orientation[
                                         1] * np.cos(heightmap_rotation_angle)])

        # Move gripper to location above pushing point
        pushing_point_margin = 0.1
        location_above_pushing_point = (position[0], position[1], position[2] + pushing_point_margin)

        # Compute gripper position and linear movement increments
        tool_position = location_above_pushing_point
        sim_ret, UR5_target_position = vrep.simxGetObjectPosition(self.sim_client, self.UR5_target_handle, -1,
                                                                  vrep.simx_opmode_blocking)
        move_direction = np.asarray(
            [tool_position[0] - UR5_target_position[0], tool_position[1] - UR5_target_position[1],
             tool_position[2] - UR5_target_position[2]])
        move_magnitude = np.linalg.norm(move_direction)
        move_step = 0.05 * move_direction / move_magnitude
        num_move_steps = int(np.floor(move_direction[0] / move_step[0]))

        # Compute gripper orientation and rotation increments
        sim_ret, gripper_orientation = vrep.simxGetObjectOrientation(self.sim_client, self.UR5_target_handle, -1,
                                                                     vrep.simx_opmode_blocking)
        rotation_step = 0.3 if (tool_rotation_angle - gripper_orientation[1] > 0) else -0.3
        num_rotation_steps = int(np.floor((tool_rotation_angle - gripper_orientation[1]) / rotation_step))

        # Simultaneously move and rotate gripper
        for step_iter in range(max(num_move_steps, num_rotation_steps)):
            vrep.simxSetObjectPosition(self.sim_client, self.UR5_target_handle, -1, (
            UR5_target_position[0] + move_step[0] * min(step_iter, num_move_steps),
            UR5_target_position[1] + move_step[1] * min(step_iter, num_move_steps),
            UR5_target_position[2] + move_step[2] * min(step_iter, num_move_steps)), vrep.simx_opmode_blocking)
            vrep.simxSetObjectOrientation(self.sim_client, self.UR5_target_handle, -1, (
            np.pi / 2, gripper_orientation[1] + rotation_step * min(step_iter, num_rotation_steps), np.pi / 2),
                                          vrep.simx_opmode_blocking)
        vrep.simxSetObjectPosition(self.sim_client, self.UR5_target_handle, -1,
                                   (tool_position[0], tool_position[1], tool_position[2]),
                                   vrep.simx_opmode_blocking)
        vrep.simxSetObjectOrientation(self.sim_client, self.UR5_target_handle, -1,
                                      (np.pi / 2, tool_rotation_angle, np.pi / 2), vrep.simx_opmode_blocking)

        # Ensure gripper is closed
        self.close_gripper()

        # Approach pushing point
        self.move_to(position, None)

        # Compute target location (push to the right)
        push_length = 0.1
        target_x = min(max(position[0] + push_direction[0] * push_length, workspace_limits[0][0]),
                       workspace_limits[0][1])
        target_y = min(max(position[1] + push_direction[1] * push_length, workspace_limits[1][0]),
                       workspace_limits[1][1])
        push_length = np.sqrt(np.power(target_x - position[0], 2) + np.power(target_y - position[1], 2))

        # Move in pushing direction towards target location
        self.move_to([target_x, target_y, position[2]], None)

        # Move gripper to location above grasp target
        self.move_to([target_x, target_y, location_above_pushing_point[2]], None)

        push_success = True

        return push_success

    def get_obj_positions(self):

        obj_positions = []
        for object_handle in self.object_handles:
            sim_ret, object_position = vrep.simxGetObjectPosition(self.sim_client, object_handle, -1, vrep.simx_opmode_blocking)
            obj_positions.append(object_position)

        return obj_positions


#
# Programme de test
#
if __name__ == '__main__':
    # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
    workspace_limits = np.asarray([[-0.724, -0.276], [-0.224, 0.224], [-0.0001, 0.4]])
    IP_PF_GEI_O4 = '10.31.24.134'
    IP_KOALA = '192.168.1.19'
    robot = RobotSim('objects/blocks', workspace_limits, True,ip_vrep=IP_KOALA)
    #robot = RobotSim('objects/blocks', workspace_limits, True)  # Pour faire tourner la simu sur la même machine que V-REP
    robot.move_to([-0.5, 0, 0])
    robot.close_gripper()
    robot.move_to([-0.5, 0, 0.5])
    robot.open_gripper()