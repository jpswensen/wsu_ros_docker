from __future__ import annotations

import os
import time
import math
import threading
import argparse
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import scipy.optimize as opt

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from tf_transformations import quaternion_from_euler


from ME583_Helpers_F25 import *

if __name__ == "__main__":

    # Initialize ROS once
    if not rclpy.ok():
        rclpy.init(args=None)
    node = FloatingCamClient()
    node.start()
    time.sleep(0.5)

    # Image acquisition
    imgG = None
    img = None
    # Goal pose: origin (match camera_demo)
    origin_pos = (0.0, 0.0, -7.0)
    origin_rpy = (0.0, 0.0, 0.0)
    node.publish_wait_pose_rpy(origin_pos, origin_rpy)
    time.sleep(1.0)
    imgG = node.wait_for_image(timeout=8.0)

    g_oc_goal = node.get_pose()
    print(f'Goal pose:\n{g_oc_goal}')

    # Initial pose: compose relative HW4 motion with origin pose (R0=I)
    initial_pos = (-1.0, -1.0, -10.0)
    # initial_rpy = (-np.pi/28, np.pi/20, 0.0)
    initial_rpy = (-np.pi/28, 0.0, np.pi/20)
    node.publish_wait_pose_rpy(initial_pos, initial_rpy)

    time.sleep(1.0)
    img = node.wait_for_image(timeout=8.0)

    g_oc_initial = node.get_pose()
    print(f'Initial pose:\n{g_oc_initial}')


    cv2.imwrite('GoalImage.png', imgG)
    cv2.imwrite('CurrentImage.png', img)
    print('Saved images: GoalImage.png, CurrentImage.png')


    xgM, xM = findMatchedPointsByColor(imgG, img)
    print(f'Matched by color: {xgM.shape[0]} pairs')
    for i in range(xgM.shape[0]):
        print(f'  Goal: {xgM[i,:]}, Current: {xM[i,:]}')

    # Build K (HW4 parameters)
    H, W = imgG.shape[:2]
    print(f'Image size W x H = {W} x {H}')
    f = 0.035  # meters
    sensor_width = 0.032
    sensor_height = sensor_width * 9.0/16.0
    sx = W / sensor_width
    sy = H / sensor_height
    st = 0.0
    ox = W/2.0
    oy = H/2.0
    K = K_intrinsic(f, sx, sy, st, ox, oy)

    xG_pix = to_homog_xy(xgM).astype(float)
    x_pix = to_homog_xy(xM).astype(float)
    Kinv = np.linalg.inv(K)
    xG_cam = Kinv @ xG_pix
    x_cam = Kinv @ x_pix


    g_oc_current = node.get_pose()
    g_goal_camera = g_oc_goal.inv @ g_oc_current
    g_camera_goal = g_goal_camera.inv
    
    print(f'Goal to current camera pose:\n{g_goal_camera}')

    pntsG,pnts=triangulate_points(xG_cam, x_cam, g_goal_camera.R, g_goal_camera.p)
    # pntsG,pnts = triangulate_points_algebraic(xG_cam, x_cam, g12.R, g12.p)
    print(f'Triangulated {pntsG.shape[1]} points.')
    print(pntsG.T)


    ## Now, iterate and calculate the velocity to go back to the goal
    for I in range(300):

        img = node.wait_for_image(timeout=8.0)
        
        xgM, xM = findMatchedPointsByColor(imgG, img)
        xG_pix = to_homog_xy(xgM).astype(float)
        x_pix = to_homog_xy(xM).astype(float)
        Kinv = np.linalg.inv(K)
        xG_cam = Kinv @ xG_pix
        x_cam = Kinv @ x_pix


        # Compute the current pose of the camera relative to the goal        
        g_oc_current = node.get_pose()
        g_goal_camera = g_oc_goal.inv @ g_oc_current
        g_camera_goal = g_goal_camera.inv


        # triangulate points again with current pose
        pntsGLoop,pnts=triangulate_points(xG_cam, x_cam, g_goal_camera.R, g_goal_camera.p)
        pntsG = pntsGLoop

        print(f'Triangulated {pntsG.shape[1]} points at iteration {I}.')
        print(pntsG.T)

        # TODO: Compute the interaction matrix at the current pose
        # TODO: Compute the feature error at the current pose
        # TODO: Compute the velocity command to move back to the goal (for now just zeros)
        Vb = np.zeros((6,1))

        # only step the camera for a short time
        node.set_camera_velocity(Vb[0:3], Vb[3:6])
        time.sleep(0.05)
        node.set_camera_velocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    # Set zero velocity at end
    node.set_camera_velocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    time.sleep(1.0)


    endimg = node.wait_for_image(timeout=8.0)

    cv2.imwrite('EndImage.png', endimg)

    # Cleanup ROS
    try:
        node.stop()
        node.destroy_node()
    except Exception as e:
        print('Cleanup warning:', e)
    if rclpy.ok():
        rclpy.shutdown()