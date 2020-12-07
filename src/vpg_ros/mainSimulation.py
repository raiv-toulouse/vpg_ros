#!/usr/bin/env python
# coding: utf-8

import time
import os
import threading
import numpy as np
import cv2
import rospy
import array
from vpg_ros.trainer import Trainer
from vpg_ros.logger import Logger
from vpg_ros.utils import *
from vpg_ros.srv import CoordAction, InfoCamera
from vpg_ros.srv import ColorDepthImages
from vpg_ros.srv import AddObjects
from std_srvs.srv import Empty

class Simulation:
    def __init__(self, args):
        print('coucou')
        # --------------- Setup options ---------------
        self.workspace_limits = np.asarray([[-0.724, -0.276], [-0.224, 0.224], [-0.0001, 0.4]])  # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
        self.heightmap_resolution = args.heightmap_resolution  # Meters per pixel of heightmap
        self.num_obj = args.num_obj  # Nb of objects
        # ------------- Algorithm options -------------
        future_reward_discount = args.future_reward_discount
        explore_rate_decay = args.explore_rate_decay
        # ------ Pre-loading and logging options ------
        logging_directory = os.path.abspath(args.logging_directory)
        save_visualizations = args.save_visualizations  # Save visualizations of FCN predictions? Takes 0.6s per training step if set to True
        # Set random seed
        np.random.seed(1234)
        # Setup virtual camera in simulation
        self.get_camera_informations()
        # Create objects and add them to VREP
        self.create_objects(self.num_obj)
        # Initialize trainer
        self.trainer = Trainer(future_reward_discount)
        # Initialize data logger
        self.logger = Logger(logging_directory)
        # logger.save_camera_info(robot.cam_intrinsics, robot.cam_pose, robot.cam_depth_scale) # Save camera intrinsics and pose
        self.logger.save_heightmap_info(self.workspace_limits, self.heightmap_resolution)  # Save heightmap parameters
        # Initialize variables for heuristic bootstrapping and exploration probability
        self.no_change_count = [2, 2]
        explore_prob = 0.5
        # Quick hack for nonlocal memory between threads in Python 2
        self.nonlocal_variables = {'executing_action': False,
                                   'primitive_action': None,
                                   'best_pix_ind': None,
                                   'push_success': False,
                                   'grasp_success': False}

        # Parallel thread to process network output and execute actions
        # -------------------------------------------------------------
        def process_actions():
            while True:
                if self.nonlocal_variables['executing_action']:
                    # Determine whether grasping or pushing should be executed based on network predictions
                    best_push_conf = np.max(self.push_predictions)
                    best_grasp_conf = np.max(self.grasp_predictions)
                    print('Primitive confidence scores: %f (push), %f (grasp)' % (best_push_conf, best_grasp_conf))
                    self.nonlocal_variables['primitive_action'] = 'grasp'
                    explore_actions = False
                    if best_push_conf > best_grasp_conf:
                        self.nonlocal_variables['primitive_action'] = 'push'
                    explore_actions = np.random.uniform() < explore_prob
                    if explore_actions:  # Exploitation (do best action) vs exploration (do other action)
                        print('Strategy: explore (exploration probability: %f)' % (explore_prob))
                        self.nonlocal_variables['primitive_action'] = 'push' if np.random.randint(0, 2) == 0 else 'grasp'
                    else:
                        print('Strategy: exploit (exploration probability: %f)' % (explore_prob))
                    self.trainer.is_exploit_log.append([0 if explore_actions else 1])
                    self.logger.write_to_log('is-exploit', self.trainer.is_exploit_log)
                    # If heuristic bootstrapping is enabled: if change has not been detected more than 2 times, execute heuristic algorithm to detect grasps/pushes
                    # NOTE: typically not necessary and can reduce final performance.

                    use_heuristic = False
                    # Get pixel location and rotation with highest affordance prediction from heuristic algorithms (rotation, y, x)
                    if self.nonlocal_variables['primitive_action'] == 'push':
                        self.nonlocal_variables['best_pix_ind'] = np.unravel_index(np.argmax(self.push_predictions), self.push_predictions.shape)
                        predicted_value = np.max(self.push_predictions)
                    elif self.nonlocal_variables['primitive_action'] == 'grasp':
                        self.nonlocal_variables['best_pix_ind'] = np.unravel_index(np.argmax(self.grasp_predictions), self.grasp_predictions.shape)
                        predicted_value = np.max(self.grasp_predictions)
                    self.trainer.use_heuristic_log.append([0])
                    self.logger.write_to_log('use-heuristic', self.trainer.use_heuristic_log)
                    # Save predicted confidence value
                    self.trainer.predicted_value_log.append([predicted_value])
                    self.logger.write_to_log('predicted-value', self.trainer.predicted_value_log)
                    # Compute 3D position of pixel
                    print('Action: %s at (%d, %d, %d)' % (
                    self.nonlocal_variables['primitive_action'], self.nonlocal_variables['best_pix_ind'][0], self.nonlocal_variables['best_pix_ind'][1],
                    self.nonlocal_variables['best_pix_ind'][2]))
                    best_rotation_angle = np.deg2rad(self.nonlocal_variables['best_pix_ind'][0] * (360.0 / self.trainer.model.num_rotations))
                    best_pix_x = self.nonlocal_variables['best_pix_ind'][2]
                    best_pix_y = self.nonlocal_variables['best_pix_ind'][1]
                    primitive_position = [best_pix_x * self.heightmap_resolution + self.workspace_limits[0][0],
                                          best_pix_y * self.heightmap_resolution + self.workspace_limits[1][0],
                                          self.valid_depth_heightmap[best_pix_y][best_pix_x] + self.workspace_limits[2][0]]
                    # If pushing, adjust start position, and make sure z value is safe and not too low
                    if self.nonlocal_variables['primitive_action'] == 'push':  # or self.nonlocal_variables['primitive_action'] == 'place':
                        finger_width = 0.02
                        safe_kernel_width = int(np.round((finger_width / 2) / self.heightmap_resolution))
                        local_region = self.valid_depth_heightmap[
                                       max(best_pix_y - safe_kernel_width, 0):min(best_pix_y + safe_kernel_width + 1, self.valid_depth_heightmap.shape[0]),
                                       max(best_pix_x - safe_kernel_width, 0):min(best_pix_x + safe_kernel_width + 1, self.valid_depth_heightmap.shape[1])]
                        if local_region.size == 0:
                            safe_z_position = self.workspace_limits[2][0]
                        else:
                            safe_z_position = np.max(local_region) + self.workspace_limits[2][0]
                        primitive_position[2] = safe_z_position
                    # Save executed primitive
                    if self.nonlocal_variables['primitive_action'] == 'push':
                        self.trainer.executed_action_log.append(
                            [0, self.nonlocal_variables['best_pix_ind'][0], self.nonlocal_variables['best_pix_ind'][1], self.nonlocal_variables['best_pix_ind'][2]])  # 0 - push
                    elif self.nonlocal_variables['primitive_action'] == 'grasp':
                        self.trainer.executed_action_log.append(
                            [1, self.nonlocal_variables['best_pix_ind'][0], self.nonlocal_variables['best_pix_ind'][1], self.nonlocal_variables['best_pix_ind'][2]])  # 1 - grasp
                    self.logger.write_to_log('executed-action', self.trainer.executed_action_log)
                    # Visualize executed primitive, and affordances
                    if save_visualizations:
                        push_pred_vis = self.trainer.get_prediction_vis(self.push_predictions, self.color_heightmap, self.nonlocal_variables['best_pix_ind'])
                        self.logger.save_visualizations(self.trainer.iteration, push_pred_vis, 'push')
                        cv2.imwrite('visualization.push.png', push_pred_vis)
                        grasp_pred_vis = self.trainer.get_prediction_vis(self.grasp_predictions, self.color_heightmap, self.nonlocal_variables['best_pix_ind'])
                        self.logger.save_visualizations(self.trainer.iteration, grasp_pred_vis, 'grasp')
                        cv2.imwrite('visualization.grasp.png', grasp_pred_vis)
                    # Initialize variables that influence reward
                    self.nonlocal_variables['push_success'] = False
                    self.nonlocal_variables['grasp_success'] = False
                    change_detected = False
                    # Execute primitive
                    if self.nonlocal_variables['primitive_action'] == 'push':
                        self.nonlocal_variables['push_success'] = self.push(primitive_position, best_rotation_angle)
                        print('Push successful: %r' % (self.nonlocal_variables['push_success']))
                    elif self.nonlocal_variables['primitive_action'] == 'grasp':
                        self.nonlocal_variables['grasp_success'] = self.grasp(primitive_position, best_rotation_angle)
                        print('Grasp successful: %r' % (self.nonlocal_variables['grasp_success']))
                    self.nonlocal_variables['executing_action'] = False
                time.sleep(0.01)

        action_thread = threading.Thread(target=process_actions)
        action_thread.daemon = True
        action_thread.start()
        self.exit_called = False
        # -------------------------------------------------------------

    def execute(self):
        # -------------------------------------------------------------
        # Start main training/testing loop
        while True:
            print('\n%s iteration: %d' % ('Training', self.trainer.iteration))
            iteration_time_0 = time.time()
            # Make sure simulation is still stable (if not, reset simulation)
            # robot.check_sim()
            # Get latest RGB-D image
            color_img, depth_img = self.get_camera_data()
            depth_img = depth_img * self.cam_depth_scale  # Apply depth scale from calibration
            # cv2.imshow('photo',depth_img)
            # if cv2.waitKey(0)==27:
            #     continue
            # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
            self.color_heightmap, self.depth_heightmap = get_heightmap(color_img, depth_img, self.cam_intrinsics, self.cam_pose, self.workspace_limits, self.heightmap_resolution)
            self.valid_depth_heightmap = self.depth_heightmap.copy()
            self.valid_depth_heightmap[np.isnan(self.valid_depth_heightmap)] = 0
            # Save RGB-D images and RGB-D heightmaps
            self.logger.save_images(self.trainer.iteration, color_img, depth_img, '0')
            self.logger.save_heightmaps(self.trainer.iteration, self.color_heightmap, self.valid_depth_heightmap, '0')
            # Reset simulation or pause real-world training if table is empty
            stuff_count = np.zeros(self.valid_depth_heightmap.shape)
            stuff_count[self.valid_depth_heightmap > 0.02] = 1
            empty_threshold = 300
            if np.sum(stuff_count) < empty_threshold or (self.no_change_count[0] + self.no_change_count[1] > 10):
                self.no_change_count = [0, 0]
                print('Not enough objects in view (value: %d)! Repositioning objects.' % (np.sum(stuff_count)))
                self.restart_sim()
                self.create_objects(self.num_obj)
                self.trainer.clearance_log.append([self.trainer.iteration])
                self.logger.write_to_log('clearance', self.trainer.clearance_log)
                continue
            if not self.exit_called:
                # Run forward pass with network to get affordances
                self.push_predictions, self.grasp_predictions, state_feat = self.trainer.forward(self.color_heightmap, self.valid_depth_heightmap, is_volatile=True)
                # Execute best primitive action on robot in another thread
                self.nonlocal_variables['executing_action'] = True
            # Run training iteration in current thread (aka training thread)
            if 'prev_color_img' in locals():
                # Detect changes
                depth_diff = abs(self.depth_heightmap - prev_depth_heightmap)
                depth_diff[np.isnan(depth_diff)] = 0
                depth_diff[depth_diff > 0.3] = 0
                depth_diff[depth_diff < 0.01] = 0
                depth_diff[depth_diff > 0] = 1
                change_threshold = 300
                change_value = np.sum(depth_diff)
                change_detected = change_value > change_threshold or prev_grasp_success
                print('Change detected: %r (value: %d)' % (change_detected, change_value))
                if change_detected:
                    if prev_primitive_action == 'push':
                        self.no_change_count[0] = 0
                    elif prev_primitive_action == 'grasp':
                        self.no_change_count[1] = 0
                else:
                    if prev_primitive_action == 'push':
                        self.no_change_count[0] += 1
                    elif prev_primitive_action == 'grasp':
                        self.no_change_count[1] += 1
                # Compute training labels
                label_value, prev_reward_value = self.trainer.get_label_value(prev_primitive_action, prev_push_success, prev_grasp_success, change_detected, prev_push_predictions,
                                                                              prev_grasp_predictions, self.color_heightmap, self.valid_depth_heightmap)
                self.trainer.label_value_log.append([label_value])
                self.logger.write_to_log('label-value', self.trainer.label_value_log)
                self.trainer.reward_value_log.append([prev_reward_value])
                self.logger.write_to_log('reward-value', self.trainer.reward_value_log)
                # Backpropagate
                self.trainer.backprop(self.prev_color_heightmap, self.prev_valid_depth_heightmap, prev_primitive_action, prev_best_pix_ind, label_value)
                # Adjust exploration probability
                explore_prob = max(0.5 * np.power(0.9998, self.trainer.iteration), 0.1)
                # Do sampling for experience replay

                sample_primitive_action = prev_primitive_action
                if sample_primitive_action == 'push':
                    sample_primitive_action_id = 0
                    sample_reward_value = 0 if prev_reward_value == 0.5 else 0.5
                elif sample_primitive_action == 'grasp':
                    sample_primitive_action_id = 1
                    sample_reward_value = 0 if prev_reward_value == 1 else 1
                # Get samples of the same primitive but with different results
                sample_ind = np.argwhere(np.logical_and(np.asarray(self.trainer.reward_value_log)[1:self.trainer.iteration, 0] == sample_reward_value,
                                                        np.asarray(self.trainer.executed_action_log)[1:self.trainer.iteration, 0] == sample_primitive_action_id))
                if sample_ind.size > 0:
                    # Find sample with highest surprise value
                    sample_surprise_values = np.abs(np.asarray(self.trainer.predicted_value_log)[sample_ind[:, 0]] - np.asarray(self.trainer.label_value_log)[sample_ind[:, 0]])
                    sorted_surprise_ind = np.argsort(sample_surprise_values[:, 0])
                    sorted_sample_ind = sample_ind[sorted_surprise_ind, 0]
                    pow_law_exp = 2
                    rand_sample_ind = int(np.round(np.random.power(pow_law_exp, 1) * (sample_ind.size - 1)))
                    sample_iteration = sorted_sample_ind[rand_sample_ind]
                    print('Experience replay: iteration %d (surprise value: %f)' % (sample_iteration, sample_surprise_values[sorted_surprise_ind[rand_sample_ind]]))
                    # Load sample RGB-D heightmap
                    self.sample_color_heightmap = cv2.imread(os.path.join(self.logger.color_heightmaps_directory, '%06d.0.color.png' % (sample_iteration)))
                    self.sample_color_heightmap = cv2.cvtColor(self.sample_color_heightmap, cv2.COLOR_BGR2RGB)
                    sample_depth_heightmap = cv2.imread(os.path.join(self.logger.depth_heightmaps_directory, '%06d.0.depth.png' % (sample_iteration)), -1)
                    sample_depth_heightmap = sample_depth_heightmap.astype(np.float32) / 100000
                    # Compute forward pass with sample
                    sample_push_predictions, sample_grasp_predictions, sample_state_feat = self.trainer.forward(self.sample_color_heightmap, sample_depth_heightmap,
                                                                                                                is_volatile=True)
                    # Get labels for sample and backpropagate
                    sample_best_pix_ind = (np.asarray(self.trainer.executed_action_log)[sample_iteration, 1:4]).astype(int)
                    self.trainer.backprop(self.sample_color_heightmap, sample_depth_heightmap, sample_primitive_action, sample_best_pix_ind, sample_reward_value)
                    # Recompute prediction value
                    if sample_primitive_action == 'push':
                        self.trainer.predicted_value_log[sample_iteration] = [np.max(sample_push_predictions)]
                    elif sample_primitive_action == 'grasp':
                        self.trainer.predicted_value_log[sample_iteration] = [np.max(sample_grasp_predictions)]
                else:
                    print('Not enough prior training samples. Skipping experience replay.')
                # Save model snapshot
                self.logger.save_backup_model(self.trainer.model)
                if self.trainer.iteration % 50 == 0:
                    self.logger.save_model(self.trainer.iteration, self.trainer.model)
                    if self.trainer.use_cuda:
                        self.trainer.model = self.trainer.model.cuda()
            # Sync both action thread and training thread
            while self.nonlocal_variables['executing_action']:
                time.sleep(0.01)
            if self.exit_called:
                break
            # Save information for next training step
            prev_color_img = color_img.copy()
            prev_depth_img = depth_img.copy()
            self.prev_color_heightmap = self.color_heightmap.copy()
            prev_depth_heightmap = self.depth_heightmap.copy()
            self.prev_valid_depth_heightmap = self.valid_depth_heightmap.copy()
            prev_push_success = self.nonlocal_variables['push_success']
            prev_grasp_success = self.nonlocal_variables['grasp_success']
            prev_primitive_action = self.nonlocal_variables['primitive_action']
            prev_push_predictions = self.push_predictions.copy()
            prev_grasp_predictions = self.grasp_predictions.copy()
            prev_best_pix_ind = self.nonlocal_variables['best_pix_ind']
            self.trainer.iteration += 1
            iteration_time_1 = time.time()
            print('Time elapsed: %f' % (iteration_time_1 - iteration_time_0))

    # Méthodes faisant le lien avec les autres objets en appelant leurs services

    def restart_sim(self):
        '''
        Appel au service restart_sim du node robot
        :return:
        '''
        rospy.wait_for_service('restart_sim')
        try:
            cmdRestartSim = rospy.ServiceProxy('restart_sim', Empty)
            resp = cmdRestartSim()
            if resp:
                print('Fin de restart_sim')
        except rospy.ServiceException as e:
            print("Service restart_sim call failed: %s" % e)

    def push(self, position, angle):
        '''
        Appel au service robot_push du node robot
        :param position:
        :param angle:
        :return:
        '''
        rospy.wait_for_service('robot_push')
        try:
            cmdPush = rospy.ServiceProxy('robot_push', CoordAction)
            resp = cmdPush(position, angle)
            return resp.success
        except rospy.ServiceException as e:
            print("Service robot_push call failed: %s" % e)

    def grasp(self, position, angle):
        '''
        Appel au service robot_grasp du node robot
        :param position:
        :param angle:
        :return:
        '''
        rospy.wait_for_service('robot_grasp')
        try:
            cmdGrasp = rospy.ServiceProxy('robot_grasp', CoordAction)
            resp = cmdGrasp(position, angle)
            print('===============> robot_grasp : {},{} => {}'.format(position, angle, resp))
            return resp.success
        except rospy.ServiceException as e:
            print("Service robot_grasp call failed: %s" % e)

    def get_camera_data(self):
        rospy.wait_for_service('get_color_depth_images')
        try:
            getImages = rospy.ServiceProxy('get_color_depth_images', ColorDepthImages)
            resp = getImages()
            width = resp.width
            height = resp.height
            resp.colorImage = list(array.array("B", resp.colorImage))
            color_image = np.asarray(resp.colorImage, dtype=np.uint8).reshape(width, height, 3)
            depth_image = np.asarray(resp.depthImage, dtype=np.float64).reshape(width, height)
            return color_image, depth_image
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def get_camera_informations(self):
        rospy.wait_for_service('get_camera_informations')
        try:
            getInfos = rospy.ServiceProxy('get_camera_informations', InfoCamera)
            resp = getInfos()
            self.cam_depth_scale = resp.depth_scale
            self.cam_intrinsics = np.asarray(resp.intrinsics, dtype=np.float64).reshape(3, 3)
            self.cam_pose = np.asarray(resp.pose, dtype=np.float64).reshape(4, 4)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def create_objects(self,nb_obj):
        rospy.wait_for_service('add_objects')
        try:
            addObjects = rospy.ServiceProxy('add_objects', AddObjects)
            addObjects(nb_obj)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
