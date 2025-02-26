#!/usr/bin/python3.8

import sys
import copy

import rospy, cv2
import ros_numpy
from sensor_msgs.msg import Image
import subprocess
import os
from os.path import exists

from termcolor import colored

import moveit_commander

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String

import baxter_interface

from yolov7_package import Yolov7Detector

import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

import matplotlib.image
import matplotlib.pyplot as plt
import numpy as np
import math
import PIL

from timeit import default_timer as timer

import random

# importing Python multiprocessing module
# import multiprocessing
# import multiprocess
import threading

import cv_bridge


def send_image(path):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)


def set_robotarm(object_select):
    global robotarm
    # if robotarm == 'left':
    if object_select == 'bowl' or object_select == 'book':
        robotarm = 'left'
        pickplace = Pick_and_Place('left_arm','left')
        pickplace.input_pos("/home/rrl/ros_ws_baxter/position_leftarm.txt")
    else:
        robotarm = 'right'
        pickplace = Pick_and_Place('right_arm','right')
        pickplace.input_pos("/home/rrl/ros_ws_baxter/position_rightarm.txt")


class Pick_and_Place(object):


  def __init__(self, limb, side):

  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.

    self.robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    self.scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    # self.group = moveit_commander.MoveGroupCommander("left_arm")
    # self.group.set_max_velocity_scaling_factor(0.7);
    #
    # self.gripper = baxter_interface.Gripper("left")
    # self.gripper.calibrate()
    # self.gripper.set_holding_force(100.0)

    global robotarm
    if robotarm == 'left':
        self.group = moveit_commander.MoveGroupCommander("left_arm")
        self.group.set_max_velocity_scaling_factor(0.5);
        self.gripper = baxter_interface.Gripper("left")
        self.gripper.calibrate()
        self.gripper.set_holding_force(100.0)
    else:
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        self.group.set_max_velocity_scaling_factor(0.5);
        self.gripper = baxter_interface.Gripper("right")
        self.gripper.calibrate()
        self.gripper.set_holding_force(100.0)


    self.object_pick_pose_target = dict()
    self.object_place_pose_target = dict()
    # self.marker = dict()

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.

    # self.pub_x = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    # display_trajectory_publisher = rospy.Publisher(
    #                                   '/move_group/display_planned_path',
    #                                   moveit_msgs.msg.DisplayTrajectory,queue_size=10)

    ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print("============ Waiting for RVIZ...")
    print("============ Starting tutorial ")

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## We can get the name of the reference frame for this robot
    print("============ Reference frame: %s" % self.group.get_planning_frame() )
    self.group.set_pose_reference_frame("pedestal")

    ## We can also print the name of the end-effector link for this group
    print("============ Reference frame: %s" % self.group.get_end_effector_link() )

    ## We can get a list of all the groups in the robot
    print("============ Robot Groups:")
    print(self.robot.get_group_names() )





  #def pick_place_implementation(self,pose_target):
  def pick_place_implementation(self):
    print("============ picking 1")

    index = 0

    # global x_center_scissors, y_center_scissors

    while True:
      if index == 1:
        break
      else:

        # if exit_flag:
        #     break

        global robotarm, object_select

        print(colored('object_select', 'green'))
        print(colored(object_select, 'green'))


        if robotarm == 'left':


            print("pose_untuck")
            pose_untuck = copy.deepcopy(self.object_pick_pose_target[index])
            # Go to Untuck
            # 0.5808682516063088,0.18265627129833253,0.09962792905394961
            # 0.14304418611557945,0.9894046523640374,0.008753868447158444,0.023241438986002203
            pose_untuck.orientation.x = 0.14304418611557945
            pose_untuck.orientation.y = 0.9894046523640374
            pose_untuck.orientation.z = 0.008753868447158444
            pose_untuck.orientation.w = 0.023241438986002203
            pose_untuck.position.x = 0.5808682516063088
            pose_untuck.position.y = 0.18265627129833253
            pose_untuck.position.z = 0.09962792905394961

            # self.gripper.command_position(100.0)

            self.moveToPose(pose_untuck)
            rospy.sleep(1)



            # Bowl

            # Modify
            if object_select == 'bowl':

                # Check if bowl is detected yet
                while x_center_bowl == 0:
                    # print("sleep")
                    rospy.sleep(5)

                print(colored('bowl', 'blue'))

                # Change pick_pose_offset x and y position according to Computer Vision calculation
                cx = (x_center_bowl - (width_bowl/2))
                cy = y_center_bowl
                pix_size = 0.0025
                h = 1.0 # Height from table to camera, when at vision place
                height = 800
                width = 1280
                # Position of the object in the baxter's stationary base frame
                xb = ((width/2) - (cy - (height/2)))*pix_size*h
                yb = - (cx - (width/2) )*pix_size*h
                #
                # # print('cx')
                # # print(cx)
                #
                # pick_pose_offset = copy.deepcopy(self.object_pick_pose_target[index])
                #
                # if xb > 1.0:
                #     xb = xb - 0.1
                #
                # pick_pose_offset.position.x = xb - 0.1
                # pick_pose_offset.position.y = yb
                #
                # # print("xb and yb bowl")
                # # print(xb)
                # # print(yb)
                #
                # # pick_pose_offset.position.z = pick_pose_offset.position.z
                # # print("pick_pose_offset")
                # # print(pick_pose_offset)
                # rospy.sleep(3)
                #
                # self.moveToPose(pick_pose_offset)
                #
                # print("============ grasping open")
                #
                # self.gripper.command_position(100.0)
                #
                # rospy.sleep(1)
                #
                # print("============ picking 2")
                # print("pose_target")
                #
                # pick_pose = copy.deepcopy(self.object_pick_pose_target[index])
                #
                # pick_pose.position.x = xb - 0.1
                #
                # # print("xb and yb bowl")
                # # print(xb)
                # # print(yb)
                #
                # self.moveToPose(pick_pose)
                #
                # print("============ grasping close")
                #
                # self.gripper.command_position(0.0)
                #
                # rospy.sleep(1)
                #
                # print("============ placing 1")
                #
                # place_pose_offset = copy.deepcopy(self.object_place_pose_target[index])
                #
                # place_pose_offset.position.z = place_pose_offset.position.z + 0.2
                # # print("place_pose_offset")
                # # print(place_pose_offset)
                # rospy.sleep(1)
                #
                # self.moveToPose(place_pose_offset)
                #
                # print("============ placing 2")
                # # print("place_pose_target")
                # # print(self.object_place_pose_target[index])
                #
                # self.moveToPose(self.object_place_pose_target[index])
                #
                # self.gripper.command_position(100.0)
                #
                # print("place_pose_offset")
                # # print(place_pose_offset)
                # # print("++++++++++++ going back")
                #
                # self.moveToPose(place_pose_offset)
                # rospy.sleep(1)

                # # If out of reach for robotarm
                while xb > 0.95:
                    xb = xb - 0.05
                    # yb = yb - 0.01

                pick_pose_offset = copy.deepcopy(self.object_pick_pose_target[index])
                #
                pick_pose_offset.position.x = xb
                pick_pose_offset.position.y = yb
                # pick_pose_offset.position.z = pick_pose_offset.position.z + 0.02

                # rotate to easier grasp object
                if xb > 0.85:
                    place_pose_offset = copy.deepcopy(self.object_place_pose_target[index])
                    pick_pose_offset.orientation.x = place_pose_offset.orientation.x
                    pick_pose_offset.orientation.y = place_pose_offset.orientation.y
                    pick_pose_offset.orientation.z = place_pose_offset.orientation.z
                    pick_pose_offset.orientation.w = place_pose_offset.orientation.w
                #     pick_pose_offset.orientation.x = 0.3
                #     pick_pose_offset.orientation.y = -0.7
                #     pick_pose_offset.orientation.z = 0.6
                #     pick_pose_offset.orientation.w = -0.2

                print("xb and yb book")
                print(xb)
                print(yb)

                # pick_pose_offset.position.z = pick_pose_offset.position.z
                # print("pick_pose_offset")
                # print(pick_pose_offset)
                # rospy.sleep(3)

                # # https://github.com/ravijo/baxter_moveit_tutorial/blob/master/scripts/example.py
                # group.set_pose_target(pick_pose_offset, end_effector_link='right_gripper')

                self.moveToPose(pick_pose_offset)
                rospy.sleep(3)

                print("============ grasping open")

                self.gripper.command_position(100.0)

                rospy.sleep(1)

                print("============ picking 2")
                # print("pose_target")
                # print(self.object_pick_pose_target[index])

                pick_pose = copy.deepcopy(self.object_pick_pose_target[index])

                pick_pose.position.x = xb
                # pick_pose.position.y = yb
                # pick_pose.position.z = pick_pose.position.z + 0.02

                # rotate to easier grasp object
                if xb > 0.85:
                    place_pose_offset = copy.deepcopy(self.object_place_pose_target[index])
                    pick_pose.orientation.x = place_pose_offset.orientation.x
                    pick_pose.orientation.y = place_pose_offset.orientation.y
                    pick_pose.orientation.z = place_pose_offset.orientation.z
                    pick_pose.orientation.w = place_pose_offset.orientation.w
                    # move diagonally to object
                    pick_pose.position.x = pick_pose.position.x + (yb-pick_pose.position.y)
                    print('move x and y')
                    print(pick_pose.position.x-xb)
                    print(yb-pick_pose.position.y)

                # print("xb and yb book")
                # print(xb)
                # print(yb)

                # self.moveToPose(self.object_pick_pose_target[index])

                # group.set_pose_target(pick_pose, end_effector_link='right_gripper')

                self.moveToPose(pick_pose)
                rospy.sleep(3)

                print("============ grasping close")

                self.gripper.command_position(0.0)

                rospy.sleep(1)

                # print("pick_pose_offset")
                # print(pick_pose_offset)
                # print("++++++++++++ going back")
                #
                # # self.marker_delete(index)
                #
                # self.moveToPose(pick_pose_offset)

                # only change y position to move horizontally away from objects
                print("============ placing 1")

                place_pose_offset = copy.deepcopy(self.object_place_pose_target[index])

                # place_pose_offset.position.z = place_pose_offset.position.z + 0.2
                place_pose_offset.position.x = xb
                place_pose_offset.position.z = pick_pose.position.z
                # pick_pose_offset.position.y = yb

                print("place_pose_offset")
                print(place_pose_offset)

                self.moveToPose(place_pose_offset)
                rospy.sleep(1)

                # move to place pose
                print("============ placing 2")
                # print("place_pose_target")
                # print(self.object_place_pose_target[index])

                # self.moveToPose(self.object_place_pose_target[index])

                place_pose_offset = copy.deepcopy(self.object_place_pose_target[index])

                # place_pose_offset.position.x = pick_pose.position.x
                place_pose_offset.position.x = xb

                self.moveToPose(place_pose_offset)
                rospy.sleep(1)

                self.gripper.command_position(100.0)

                print("============ placing 3")
                # # move horizontally away to release book
                # place_pose_offset = copy.deepcopy(self.object_place_pose_target[index])
                # place_pose_offset.position.y = place_pose_offset.position.y + 0.1
                #
                # self.moveToPose(place_pose_offset)
                # rospy.sleep(3)

                # self.marker_add(index,self.object_place_pose_target[index])

                # print("place_pose_offset")
                # print(place_pose_offset)
                # print("++++++++++++ going back")

                # self.moveToPose(place_pose_offset)
                # rospy.sleep(1)

                # place_pose_offset = copy.deepcopy(self.object_place_pose_target[index])
                #
                # # rotate to release book
                # pose_untuck.position.x = place_pose_offset.position.x
                # pose_untuck.position.y = place_pose_offset.position.y
                # pose_untuck.position.z = place_pose_offset.position.z + 0.2
                #
                # print(pose_untuck)
                #
                # self.moveToPose(pose_untuck)
                # rospy.sleep(1)

                print("pose_untuck")
                pose_untuck = copy.deepcopy(self.object_pick_pose_target[index])
                # Go to Untuck
                # 0.5808682516063088,0.18265627129833253,0.09962792905394961
                # 0.14304418611557945,0.9894046523640374,0.008753868447158444,0.023241438986002203
                pose_untuck.orientation.x = 0.14304418611557945
                pose_untuck.orientation.y = 0.9894046523640374
                pose_untuck.orientation.z = 0.008753868447158444
                pose_untuck.orientation.w = 0.023241438986002203
                pose_untuck.position.x = 0.5808682516063088
                pose_untuck.position.y = 0.18265627129833253
                pose_untuck.position.z = 0.09962792905394961

                self.moveToPose(pose_untuck)
                rospy.sleep(1)


            # Book
            if object_select == 'book':
                # Check if book is detected yet
                while x_center_book == 0:
                    # print("sleep")
                    rospy.sleep(5)

                print(colored('book', 'blue'))

                # print("x_center_scissors")
                # print(x_center_scissors)
                # print(y_center_scissors)

                # Change pick_pose_offset x and y position according to Computer Vision calculation

                cx = (x_center_book - (width_book/2))
                # cx = x_center_book
                cy = y_center_book
                # pix_size = 0.00225 # Camera calibration (meter/pixels); Pixel size at 1 meter height of camera
                # h = 0.9 # Height from table to camera, when at vision place
                # pix_size_x = 0.0044 # Camera calibration (meter/pixels); Pixel size at 1 meter height of camera
                pix_size = 0.0025
                h = 1.0 # Height from table to camera, when at vision place
                # x0b = 0.188 # x position of camera in baxter's base frame, when at vision place
                # y0b = 0.007 # y position of camera in baxter's base frame, when at vision place
                height = 800
                width = 1280
                # Position of the object in the baxter's stationary base frame
                # xb = (cx - (height/2))*pix_size*h + x0b + x_camera_offset
                # yb = (cy - (width/2))*pix_size*h + y0b  + y_camera_offset
                # xb = ((width/2) - (cy - (height/2)))*pix_size*h + x0b
                # yb = (cx - (width/2) )*pix_size*h + y0b
                # xb = (cy - (height/2) )*pix_size_x*h
                xb = ((width/2) - (cy - (height/2)))*pix_size*h
                yb = - (cx - (width/2) )*pix_size*h

                # print('cx')
                # print(cx)

                # cx = x_center_scissors
                # cy = y_center_scissors
                # pix_size = 0.00225 # Camera calibration (meter/pixels); Pixel size at 1 meter height of camera
                # h = 0.9 # Height from table to camera, when at vision place
                # x0b = 0.188 # x position of camera in baxter's base frame, when at vision place
                # y0b = 0 # y position of camera in baxter's base frame, when at vision place
                # height = 800
                # width = 1200
                # # Position of the object in the baxter's stationary base frame
                # # xb = (cx - (height/2))*pix_size*h + x0b + x_camera_offset
                # # yb = (cy - (width/2))*pix_size*h + y0b  + y_camera_offset
                # xb = ((width/2) - (cy - (height/2)))*pix_size*h + x0b
                # yb = ((width/2) - cx )*pix_size*h + y0b

                pick_pose_offset = copy.deepcopy(self.object_pick_pose_target[index])
                #
                pick_pose_offset.position.x = xb
                pick_pose_offset.position.y = yb

                # print("xb and yb book")
                # print(xb)
                # print(yb)

                # pick_pose_offset.position.z = pick_pose_offset.position.z
                # print("pick_pose_offset")
                # print(pick_pose_offset)
                rospy.sleep(3)

                # # https://github.com/ravijo/baxter_moveit_tutorial/blob/master/scripts/example.py
                # group.set_pose_target(pick_pose_offset, end_effector_link='right_gripper')

                self.moveToPose(pick_pose_offset)

                print("============ grasping open")

                self.gripper.command_position(100.0)

                rospy.sleep(1)

                print("============ picking 2")
                # print("pose_target")
                # print(self.object_pick_pose_target[index])

                pick_pose = copy.deepcopy(self.object_pick_pose_target[index])

                pick_pose.position.x = xb
                # pick_pose.position.y = yb
                pick_pose.position.z = pick_pose.position.z - 0.02

                # print("xb and yb book")
                # print(xb)
                # print(yb)

                # self.moveToPose(self.object_pick_pose_target[index])

                # group.set_pose_target(pick_pose, end_effector_link='right_gripper')

                self.moveToPose(pick_pose)

                print("============ grasping close")

                self.gripper.command_position(0.0)

                rospy.sleep(1)

                # print("pick_pose_offset")
                # print(pick_pose_offset)
                # print("++++++++++++ going back")
                #
                # # self.marker_delete(index)
                #
                # self.moveToPose(pick_pose_offset)

                # only change y position to move horizontally away from objects
                print("============ placing 1")

                place_pose_offset = copy.deepcopy(self.object_place_pose_target[index])

                # place_pose_offset.position.z = place_pose_offset.position.z + 0.2
                place_pose_offset.position.x = pick_pose.position.x
                place_pose_offset.position.z = pick_pose.position.z
                # pick_pose_offset.position.y = yb

                # print("place_pose_offset")
                # print(place_pose_offset)

                self.moveToPose(place_pose_offset)
                rospy.sleep(1)

                # move to place pose
                print("============ placing 2")
                # print("place_pose_target")
                # print(self.object_place_pose_target[index])

                # self.moveToPose(self.object_place_pose_target[index])

                place_pose_offset = copy.deepcopy(self.object_place_pose_target[index])

                place_pose_offset.position.x = pick_pose.position.x

                self.moveToPose(place_pose_offset)
                rospy.sleep(1)

                self.gripper.command_position(100.0)

                print("============ placing 3")
                # # move horizontally away to release book
                # place_pose_offset = copy.deepcopy(self.object_place_pose_target[index])
                # place_pose_offset.position.y = place_pose_offset.position.y + 0.1
                #
                # self.moveToPose(place_pose_offset)
                # rospy.sleep(3)

                # self.marker_add(index,self.object_place_pose_target[index])

                # print("place_pose_offset")
                # print(place_pose_offset)
                # print("++++++++++++ going back")

                # self.moveToPose(place_pose_offset)
                # rospy.sleep(1)

                # rotate to release book
                pose_untuck.position.x = place_pose_offset.position.x
                pose_untuck.position.y = place_pose_offset.position.y
                pose_untuck.position.z = place_pose_offset.position.z + 0.2

                self.moveToPose(pose_untuck)
                rospy.sleep(1)

                print("pose_untuck")
                pose_untuck = copy.deepcopy(self.object_pick_pose_target[index])
                # Go to Untuck
                # 0.5808682516063088,0.18265627129833253,0.09962792905394961
                # 0.14304418611557945,0.9894046523640374,0.008753868447158444,0.023241438986002203
                pose_untuck.orientation.x = 0.14304418611557945
                pose_untuck.orientation.y = 0.9894046523640374
                pose_untuck.orientation.z = 0.008753868447158444
                pose_untuck.orientation.w = 0.023241438986002203
                pose_untuck.position.x = 0.5808682516063088
                pose_untuck.position.y = 0.18265627129833253
                pose_untuck.position.z = 0.09962792905394961

                self.moveToPose(pose_untuck)
                rospy.sleep(1)

        else:


            print("pose_untuck")
            pose_untuck = copy.deepcopy(self.object_pick_pose_target[index])
            # Go to Untuck
            # 0.5808682516063088,0.18265627129833253,0.09962792905394961
            # 0.14304418611557945,0.9894046523640374,0.008753868447158444,0.023241438986002203
            pose_untuck.orientation.x = -0.14304418611557945
            pose_untuck.orientation.y = 0.9894046523640374
            pose_untuck.orientation.z = 0.008753868447158444
            pose_untuck.orientation.w = 0.023241438986002203
            pose_untuck.position.x = 0.5808682516063088
            pose_untuck.position.y = -0.18265627129833253
            pose_untuck.position.z = 0.09962792905394961

            # self.gripper.command_position(100.0)

            self.moveToPose(pose_untuck)

            rospy.sleep(1)



            # Check if tennis ball is detected yet
            while x_center_ball == 0:
                # print("sleep")
                rospy.sleep(5)

            # print("x_center_ball")
            # print(x_center_ball)
            # print(y_center_ball)

            # Change pick_pose_offset x and y position according to Computer Vision calculation

            cx = x_center_ball
            cy = y_center_ball
            # pix_size = 0.00225 # Camera calibration (meter/pixels); Pixel size at 1 meter height of camera
            # h = 0.9 # Height from table to camera, when at vision place
            # pix_size_x = 0.0044 # Camera calibration (meter/pixels); Pixel size at 1 meter height of camera
            pix_size = 0.00235
            h = 1.0 # Height from table to camera, when at vision place
            # x0b = 0.188 # x position of camera in baxter's base frame, when at vision place
            # y0b = 0.007 # y position of camera in baxter's base frame, when at vision place
            height = 800
            width = 1280
            # Position of the object in the baxter's stationary base frame
            # xb = (cx - (height/2))*pix_size*h + x0b + x_camera_offset
            # yb = (cy - (width/2))*pix_size*h + y0b  + y_camera_offset
            # xb = ((width/2) - (cy - (height/2)))*pix_size*h + x0b
            # yb = (cx - (width/2) )*pix_size*h + y0b
            # xb = (cy - (height/2) )*pix_size_x*h
            xb = ((width/2) - (cy - (height/2)))*pix_size*h
            yb = - (cx - (width/2) )*pix_size*h

            # pick_pose_offset = copy.deepcopy(self.object_pick_pose_target[index])
            #
            # pick_pose_offset.position.x = xb
            # pick_pose_offset.position.y = yb
            #
            # pick_pose_offset.position.z = pick_pose_offset.position.z + 0.05
            # print("pick_pose_offset")
            # print(pick_pose_offset)

            # print("xb and yb ball")
            # print(xb)
            # print(yb)
            # print(pick_pose_offset.position.z)

            # self.moveToPose(pick_pose_offset)

            # rospy.sleep(3)

            print("============ grasping open")

            self.gripper.command_position(100.0)

            rospy.sleep(1)

            print("============ picking 2")
            # print("pose_target")
            # print(self.object_pick_pose_target[index])

            pick_pose = copy.deepcopy(self.object_pick_pose_target[index])

            pick_pose.position.x = xb
            pick_pose.position.y = yb
            pick_pose.position.z = pick_pose.position.z + 0.1

            # print(pick_pose.position.x)
            # print(pick_pose.position.y)
            # print(pick_pose.position.z)

            # print(pick_pose)

            # self.moveToPose(self.object_pick_pose_target[index])
            self.moveToPose(pick_pose)

            global exit_flag
            if exit_flag:
                exit_flag = False
                print(colored('exit_flag', 'red'))
                self.moveToPose(pose_untuck)
                rospy.sleep(1)
                continue

            rospy.sleep(3)

            pick_pose = copy.deepcopy(self.object_pick_pose_target[index])

            pick_pose.position.z = pick_pose.position.z + 0.02

            pick_pose.position.x = xb
            pick_pose.position.y = yb
            # pick_pose.position.z = pick_pose.position.z - 0.1

            # print(pick_pose.position.x)
            # print(pick_pose.position.y)
            # print(pick_pose.position.z)

            # print(pick_pose)

            # self.moveToPose(self.object_pick_pose_target[index])
            self.moveToPose(pick_pose)

            if exit_flag:
                exit_flag = False
                print(colored('exit_flag', 'red'))
                self.moveToPose(pose_untuck)
                rospy.sleep(1)
                continue

            print("============ grasping close")

            self.gripper.command_position(0.0)

            rospy.sleep(1)

            # print("pick_pose_offset")
            # print(pick_pose_offset)
            # print("++++++++++++ going back")
            #
            # # self.marker_delete(index)
            #
            # self.moveToPose(pick_pose_offset)

            print("============ placing 1")

            place_pose_offset = copy.deepcopy(self.object_place_pose_target[index])

            place_pose_offset.position.z = pick_pose.position.z
            # print("place_pose_offset")
            # print(place_pose_offset)
            # rospy.sleep(1)
            #
            # self.moveToPose(place_pose_offset)

            self.moveToPose(place_pose_offset)

            # if exit_flag:
            #     exit_flag = False
            #     self.moveToPose(pose_untuck)
            #     rospy.sleep(1)
            #     print(colored('exit_flag', 'red'))
            #     continue

            rospy.sleep(3)

            print("============ placing 2")
            # print("place_pose_target")
            # print(self.object_place_pose_target[index])

            place_pose_offset = copy.deepcopy(self.object_place_pose_target[index])
            self.moveToPose(self.object_place_pose_target[index])

            # if exit_flag:
            #     exit_flag = False
            #     self.moveToPose(pose_untuck)
            #     rospy.sleep(1)
            #     print(colored('exit_flag', 'red'))
            #     continue

            rospy.sleep(1)

            self.gripper.command_position(100.0)

            place_pose_offset.position.z = place_pose_offset.position.z + 0.2

            self.moveToPose(place_pose_offset)

            # if exit_flag:
            #     exit_flag = False
            #     self.moveToPose(pose_untuck)
            #     rospy.sleep(1)
            #     print(colored('exit_flag', 'red'))
            #     continue

            rospy.sleep(1)



            # self.marker_add(index,self.object_place_pose_target[index])

            # print("place_pose_offset")
            # print(place_pose_offset)
            print("++++++++++++ going back")

            self.moveToPose(place_pose_offset)
            rospy.sleep(1)


        print("pose_untuck")
        # print(pose_untuck)

        self.moveToPose(pose_untuck)
        rospy.sleep(1)

        index = index + 1

        ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

        # END_TUTORIAL

    # print("============ STOPPING")
    print(colored('============ STOPPING', 'red'))


  # def moveToPose_position_z(self,pose):
  #
  #       # define temp pose
  #   pose_target = geometry_msgs.msg.Pose()
  #
  #       # format the pose correctly
  #   #print "HELLOOOOOOOOOOO"
  #   #print pose
  #   # pose_target.orientation.x = pose.orientation.x
  #   # pose_target.orientation.y = pose.orientation.y
  #   # pose_target.orientation.z = pose.orientation.z
  #   # pose_target.orientation.w = pose.orientation.w
  #   # pose_target.position.x = pose.position.x
  #   # pose_target.position.y = pose.position.y
  #   pose_target.position.z = pose.position.z
  #
  #       # set things
  #   self.group.set_pose_target(pose_target)
  #   self.group.set_num_planning_attempts(5);
  #   self.group.set_planning_time(10.0);
  #   self.group.set_goal_position_tolerance(0.0075)
  #   self.group.set_goal_orientation_tolerance(0.0075)
  #
  #   print("\tPlanning...")
  #   plan1 = self.group.plan()
  #       # rospy.sleep(5)
  #   print("\tExecuting...")
  #   self.group.go(wait=True)


  # def input_pos(self,filename):
  #   index = 0
  #
  #
  #
  #   with open(filename, 'r') as f:
  #     while True:
  #       if index == 1:
  #         break
  #       else:
  #
  #         line = f.readline()
  #         line = line.rstrip()
  #         data = line.split(',')
  #         print(data)
  #         self.object_pick_pose_target[index] = geometry_msgs.msg.Pose()
  #         self.object_pick_pose_target[index].position.x = float(data[0])
  #         self.object_pick_pose_target[index].position.y = float(data[1])
  #         self.object_pick_pose_target[index].position.z = float(data[2])
  #
  #         line = f.readline()
  #         line = line.rstrip()
  #         data = line.split(',')
  #         print(data)
  #         self.object_pick_pose_target[index].orientation.x = float(data[0])
  #         self.object_pick_pose_target[index].orientation.y = float(data[1])
  #         self.object_pick_pose_target[index].orientation.z = float(data[2])
  #         self.object_pick_pose_target[index].orientation.w = float(data[3])
  #
  #         line = f.readline()
  #         line = line.rstrip()
  #         data = line.split(',')
  #
  #         self.object_place_pose_target[index] = geometry_msgs.msg.Pose()
  #         self.object_place_pose_target[index].position.x = float(data[0])
  #         self.object_place_pose_target[index].position.y = float(data[1])
  #         self.object_place_pose_target[index].position.z = float(data[2])
  #
  #         line = f.readline()
  #         line = line.rstrip()
  #         data = line.split(',')
  #
  #         self.object_place_pose_target[index].orientation.x = float(data[0])
  #         self.object_place_pose_target[index].orientation.y = float(data[1])
  #         self.object_place_pose_target[index].orientation.z = float(data[2])
  #         self.object_place_pose_target[index].orientation.w = float(data[3])
  #
  #         print(self.object_pick_pose_target[index])
  #         print(self.object_place_pose_target[index])
  #
  #         index = index + 1
  #
  #       # print "adding box"
  #   rospy.sleep(2)


  def moveToPose(self,pose):

    # if exit_flag:
    #     break
    # else
        # define temp pose
    pose_target = geometry_msgs.msg.Pose()

        # format the pose correctly
    #print "HELLOOOOOOOOOOO"
    #print pose
    pose_target.orientation.x = pose.orientation.x
    pose_target.orientation.y = pose.orientation.y
    pose_target.orientation.z = pose.orientation.z
    pose_target.orientation.w = pose.orientation.w
    pose_target.position.x = pose.position.x
    pose_target.position.y = pose.position.y
    pose_target.position.z = pose.position.z

        # set things
    self.group.set_pose_target(pose_target)
    self.group.set_num_planning_attempts(5);
    self.group.set_planning_time(10.0);
    self.group.set_goal_position_tolerance(0.0075)
    self.group.set_goal_orientation_tolerance(0.0075)

    print("\tPlanning...")
    plan1 = self.group.plan()
    # print("plan1")
    # print(plan1)
        # rospy.sleep(5)
    print("\tExecuting...")
    self.group.go(wait=True)
    # self.group.execute(plan1, wait=False)
    # go = self.group.go(wait=False)
    # rospy.sleep(3)

    # print("self.group.get_current_state()")
    # print(self.group.get_current_pose(self, end_effector_link='right_gripper'))
    # print(go)

    # end_effector_link = self.group.get_end_effector_link()
    # #
	# # # Get the current pose so we can add it as a waypoint
    # current_pose = self.group.get_current_pose(end_effector_link).pose
    # print(current_pose)

    # delete exit_flag because not necessary?
    # global exit_flag
    # if exit_flag:
    #     exit_flag = False
        # reset
    #     self.group.stop()
    #     print(colored('SSTOP EXECUTION!!!', 'red'))
        # print("STOP EXECUTION!!!")

    self.group.stop()

    # print("CONTINUE")
    print(colored('CONTINUE', 'red'))


  def moveToPose_position(self,pose):

    # if exit_flag:
    #     break
    # else
        # define temp pose
    pose_target = geometry_msgs.msg.Pose()

        # format the pose correctly
    #print "HELLOOOOOOOOOOO"
    #print pose
    # pose_target.orientation.x = pose.orientation.x
    # pose_target.orientation.y = pose.orientation.y
    # pose_target.orientation.z = pose.orientation.z
    # pose_target.orientation.w = pose.orientation.w
    pose_target.position.x = pose.position.x
    pose_target.position.y = pose.position.y
    pose_target.position.z = pose.position.z

        # set things
    self.group.set_pose_target(pose_target)
    self.group.set_num_planning_attempts(5);
    self.group.set_planning_time(10.0);
    self.group.set_goal_position_tolerance(0.0075)
    # self.group.set_goal_orientation_tolerance(0.0075)

    print("\tPlanning...")
    plan1 = self.group.plan()
    # print("plan1")
    # print(plan1)
        # rospy.sleep(5)
    print("\tExecuting...")
    self.group.go(wait=True)

    self.group.stop()

    # print("CONTINUE")
    print(colored('CONTINUE', 'red'))


  def input_pos(self,filename):
    index = 0



    with open(filename, 'r') as f:
      while True:
        if index == 1:
          break
        else:

          line = f.readline()
          line = line.rstrip()
          data = line.split(',')
          # print(data)
          self.object_pick_pose_target[index] = geometry_msgs.msg.Pose()
          self.object_pick_pose_target[index].position.x = float(data[0])
          self.object_pick_pose_target[index].position.y = float(data[1])
          self.object_pick_pose_target[index].position.z = float(data[2])

          line = f.readline()
          line = line.rstrip()
          data = line.split(',')
          # print(data)
          self.object_pick_pose_target[index].orientation.x = float(data[0])
          self.object_pick_pose_target[index].orientation.y = float(data[1])
          self.object_pick_pose_target[index].orientation.z = float(data[2])
          self.object_pick_pose_target[index].orientation.w = float(data[3])

          line = f.readline()
          line = line.rstrip()
          data = line.split(',')

          self.object_place_pose_target[index] = geometry_msgs.msg.Pose()
          self.object_place_pose_target[index].position.x = float(data[0])
          self.object_place_pose_target[index].position.y = float(data[1])
          self.object_place_pose_target[index].position.z = float(data[2])

          line = f.readline()
          line = line.rstrip()
          data = line.split(',')

          self.object_place_pose_target[index].orientation.x = float(data[0])
          self.object_place_pose_target[index].orientation.y = float(data[1])
          self.object_place_pose_target[index].orientation.z = float(data[2])
          self.object_place_pose_target[index].orientation.w = float(data[3])

          # print(self.object_pick_pose_target[index])
          # print(self.object_place_pose_target[index])

          index = index + 1

        # print "adding box"
    rospy.sleep(2)




def image_callback(ros_img):
    # Convert received image message to ros_numpy image
    np_img = ros_numpy.numpify(ros_img)
    # improve mediapipe hand detection by cropping image to show person only
    yolo_crop_to_person(np_img)
    try:
        cv2.imshow('Image', np_img) # display image
        cv2.waitKey(1)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    ## BAnima - enabling robot
    # rs = baxter_interface.RobotEnable()
    # rs.enable()
    # pickplace = Pick_and_Place('left_arm','left')
    # pickplace.input_pos("position_leftarm.txt")
    # # for index in range(0, 4):
    # #     pickplace.marker_add(index,pickplace.object_pick_pose_target[index]) #sending 1 in case of pick position
    # pickplace.pick_place_implementation()

def yolo_crop_to_person(np_img):

    global idx_yoloperson

    # print("time.time")
    # print(timer())

    img = cv2.cvtColor(np.array(np_img), cv2.COLOR_BGRA2BGR)
    det = Yolov7Detector(traced=False,conf_thres=0.5)
    classes, boxes, scores = det.detect(img)

    # print("classes")
    # print(classes)
    # print("boxes")
    # print(boxes)
    # print("scores")
    # print(scores)

    # find person
    # if 0 in classes:
    if any(0 in sublist for sublist in classes):
        for sublist in classes:
            index = sublist.index(0)
            # print(index)

        classes_person = [[classes[0][index]]]
        boxes_person = [[boxes[0][index]]]
        scores_person = [[scores[0][index]]]

        x_center = boxes_person[0][0][0]
        # print(x_center)
        y_center = boxes_person[0][0][1]
        # print(y_center)
        width = boxes_person[0][0][2]
        # print(width)
        height = boxes_person[0][0][3]
        # print(height)

        global y_min
        # print("y_min")
        # print(y_min)

        y_min_total = max(y_min, boxes_person[0][0][3])
        # print(y_min_total)

        crop_img = img[int(boxes_person[0][0][1]):int(800), int(boxes_person[0][0][0]):int(boxes_person[0][0][2])]

        cv2.imwrite('/home/rrl/ros_ws_baxter/yoloperson/ImageYOLO_person_cropped' + str(idx_yoloperson) + '.png', crop_img)

        # print("idx_yoloperson")
        # print(idx_yoloperson)


        # print(person detected!)
        mediapipe_persondetection()

    else:
        send_image('/home/rrl/ros_ws_baxter/src/baxter_tools/share/images/researchsdk.png')


def mediapipe_persondetection():
    # Mediapipe
    # For static images:

    # For static images:
    # IMAGE_FILES = []
    with mp_hands.Hands(
        static_image_mode=True,
        # max_num_hands=2,
        min_detection_confidence=0.5) as hands:

        global idx_yoloperson
        image = cv2.imread('/home/rrl/ros_ws_baxter/yoloperson/ImageYOLO_person_cropped' + str(idx_yoloperson) + '.png')

        # Convert the BGR image to RGB before processing.
        results = hands.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        # Print handedness and draw hand landmarks on the image.
        # print('Handedness:', results.multi_handedness)
        if results.multi_hand_landmarks:
            image_height, image_width, _ = image.shape
            image = image.copy()
            for hand_landmarks in results.multi_hand_landmarks:
              # print('hand_landmarks:', hand_landmarks)
              # print(
              #     f'Index finger tip coordinates: (',
              #     f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_width}, '
              #     f'{hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_height})'
              # )
              mp_drawing.draw_landmarks(
                  image,
                  hand_landmarks,
                  mp_hands.HAND_CONNECTIONS,
                  mp_drawing_styles.get_default_hand_landmarks_style(),
                  mp_drawing_styles.get_default_hand_connections_style())
            for hand_world_landmarks in results.multi_hand_world_landmarks:
              mp_drawing.plot_landmarks(
                hand_world_landmarks, mp_hands.HAND_CONNECTIONS, azimuth=5)

            for hand_landmarks in results.multi_hand_landmarks:

                # Check for curved middle finger
                middle_pip = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y * image_height]
                middle_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y * image_height]
                middle_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y * image_height]
                a = np.array(middle_mcp) # First
                b = np.array(middle_pip) # Mid
                c = np.array(middle_tip) # End
                radians = np.arctan2(c[1] - b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
                angle_middle_curve = np.abs(radians*180.0/np.pi)
                if angle_middle_curve > 180.0:
                    angle_middle_curve = 360-angle_middle_curve
                # print('angle_middle_curve')
                # print(angle_middle_curve)

                wrist_z = [hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x,hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y,hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].z]
                index_mcp_z = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].z]
                index_pip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].y,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].z]
                index_tip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].z]
                ba = np.array(index_mcp_z) - np.array(index_pip_z)
                bc = np.array(index_tip_z) - np.array(index_pip_z)
                cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
                angle_index_z = np.degrees(np.arccos(cosine_angle))
                # print('index z')
                # print(angle_index_z)


                wrist_z = [hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x,hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y,hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].z]
                middle_mcp_z = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].z]
                middle_pip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].z]
                middle_tip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].z]
                ba = np.array(middle_mcp_z) - np.array(middle_pip_z)
                bc = np.array(middle_tip_z) - np.array(middle_pip_z)
                cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
                angle_middle_z = np.degrees(np.arccos(cosine_angle))
                # print('middle z')
                # print(angle_middle_z)


                wrist_z = [hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x,hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y,hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].z]
                ring_mcp_z = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].y,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].z]
                ring_pip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP].y,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP].z]
                ring_tip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].z]
                ba = np.array(ring_mcp_z) - np.array(ring_pip_z)
                bc = np.array(ring_tip_z) - np.array(ring_pip_z)
                cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
                angle_ring_z = np.degrees(np.arccos(cosine_angle))
                # print('ring z')
                # print(angle_ring_z)


                wrist_z = [hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x,hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y,hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].z]
                pinky_mcp_z = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].z]
                pinky_pip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP].y,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP].z]
                pinky_tip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].z]
                ba = np.array(pinky_mcp_z) - np.array(pinky_pip_z)
                bc = np.array(pinky_tip_z) - np.array(pinky_pip_z)
                cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
                angle_pinky_z = np.degrees(np.arccos(cosine_angle))
                # print('pinky z')
                # print(angle_pinky_z)


                # Difference at 5cm
    #             The magnitude of z uses roughly the same scale as x
    #           Use half the palm size to approximate 5 cm
                thumb_tip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y ,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].z]
                distance_thumbtip_indextip = math.dist(thumb_tip_z,index_tip_z)
                # print("distance_thumbtip_indextip")
                # print(distance_thumbtip_indextip)

                thumb_ip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].x,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].y ,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].z]
                distance_wrist_imdexmcp = math.dist(wrist_z,index_mcp_z)
                # print("distance_wrist_imdexmcp/2")
                distance_wrist_imdexmcp_half = distance_wrist_imdexmcp/2
                # print(distance_wrist_imdexmcp)

                distance_thumbip_indexpip = (math.dist(thumb_ip_z,index_pip_z))
                # print("distance_thumbip_indexpip")
                # print(distance_thumbip_indexpip)

                grasp = 'None'

                global object_select
                if (abs(angle_index_z - angle_middle_z) < 35) and (abs(angle_index_z - angle_ring_z) > 35) and (abs(angle_index_z - angle_pinky_z) > 35):
                    print("Tripod")
                    grasp = 'Tripod'
                    print("Person wants to grab tennis ball")
                    if object_select == "ball":
                        object_select = random.choice( ["bowl", "book"] )

                        # global robotarm
                        # # if robotarm == 'left':
                        # if object_select == 'bowl' or object_select == 'book':
                        #     robotarm = 'left'
                        #     pickplace = Pick_and_Place('left_arm','left')
                        #     pickplace.input_pos("/home/rrl/ros_ws_baxter/position_leftarm.txt")

                        set_robotarm(object_select)

                        # proc1._Thread__stop()
                        # exitflag1 =True
                        # proc1.join()
                        # proc1.terminate()
                        # proc1.daemon = True
                        # proc1._stop.set()
                        # proc1._stop = threading.Event()
                        # proc1._stop.set()
                        # proc1._stop()
                        # print("STOPPPPPPP THIS THREAD!!!!!")
                        # https://stackoverflow.com/questions/34698926/python-threading-not-stopping
                        global exit_flag
                        exit_flag = True
                        pickplace.group.stop()
                        # break while loop with index = 0 to start from beginning
                        # print("STOP!!!!!")
                        print(colored('STOP!!!!!', 'red'))

                    send_image('/home/rrl/ros_ws_baxter/Tripod.png')
                elif (abs(angle_index_z - angle_middle_z) < 35) and (abs(angle_index_z - angle_ring_z) < 35) and (abs(angle_index_z - angle_pinky_z) > 35):
                    print("Sphere 4 Finger")
                    grasp = 'Sphere 4 Finger'
                    print("Person wants to grab tennis ball")
                    if object_select == "ball":
                        object_select = random.choice( ["bowl", "book"] )
                    send_image('/home/rrl/ros_ws_baxter/Sphere4Finger.png')
                elif (abs(angle_index_z - angle_middle_z) < 35) and (abs(angle_index_z - angle_ring_z) < 35) and (abs(angle_index_z - angle_pinky_z) < 35) and distance_thumbtip_indextip > distance_wrist_imdexmcp and distance_thumbip_indexpip > distance_wrist_imdexmcp:
                    print("Large Diameter")
                    grasp = 'Large Diameter'
                    print("Person wants to grab book/box")
                    if object_select == "book":
                        object_select = random.choice( ["bowl", "ball"] )
                    send_image('/home/rrl/ros_ws_baxter/LargeDiameter.png')
                elif (abs(angle_index_z - angle_middle_z) < 35) and (abs(angle_index_z - angle_ring_z) < 35) and (abs(angle_index_z - angle_pinky_z) < 35) and angle_middle_curve > 170:
                    print("Parallel Extension")
                    grasp = 'Parallel Extension'
                    print("Person wants to grab plate/frisbee OR book/box")
                    if object_select == "bowl" or object_select == "book":
                        object_select = random.choice( ["ball"] )
                    send_image('/home/rrl/ros_ws_baxter/ParallelExtension.png')
                elif (abs(angle_index_z - angle_middle_z)) < 35 and (abs(angle_index_z - angle_ring_z)) < 35 and (abs(angle_index_z - angle_pinky_z) < 35) and angle_middle_curve < 170:
                    print("Extension Type")
                    grasp = 'Extension Type'
                    print("Person wants to grab plate/frisbee")
                    if object_select == "bowl":
                        object_select = random.choice( ["book", "ball"] )
                    send_image('/home/rrl/ros_ws_baxter/ExtensionType.png')
                else:
                    send_image('/home/rrl/ros_ws_baxter/HumanDetected.png')


                # Plate/Frisbee: Extension Type, Parallel Extension)
                # Tennis ball: Tripod, Sphere 4-Finger
                # Book/Box: Large Diameter; Parallel Extension or Sphere 4-Finger (in this case parallel extension)

                # Compare with robot pick object to check for collision
                # If human and robot selected the same object, randomly select other object for the robot
                # object_select
                # object_select = random.choice( ["bowl", "book", "ball"] )

                # if (grasp == 'Parallel Extension') or (grasp == 'Extension Type'):
                #     print("Person wants to grab plate/frisbee")
                #     if object_select == "bowl":
                #         object_select = random.choice( ["book", "ball"] )
                # if (grasp == 'Tripod') or (grasp == 'Sphere 4 Finger'):
                #     print("Person wants to grab tennis ball")
                #     if object_select == "ball":
                #         object_select = random.choice( ["bowl", "book"] )
                # if (grasp == 'Large Diameter') or (grasp == 'Parallel Extension'):
                #     print("Person wants to grab book/box")
                #     if object_select == "bowl":
                #         object_select = random.choice( ["bowl", "ball"] )


            #     # Get coordinates
            #     thumb = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y * image_height]
            #     index = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_height]
            #     wrist = [hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y * image_height]
            #
            #     a = np.array(thumb) # First
            #     b = np.array(wrist) # Mid
            #     c = np.array(index) # End
            #
            #     radians = np.arctan2(c[1] - b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
            #     angle = np.abs(radians*180.0/np.pi)
            #
            #     if angle > 180.0:
            #         angle = 360-angle
            #
            #     print(angle)
            #
            #     thumb_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y * image_height]
            #     index_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image_height]
            #
            #     thumb_ip = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].y * image_height]
            #     index_dip = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].y * image_height]
            #
            #
            #     index_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y * image_height]
            #     a = np.array(wrist) # First
            #     b = np.array(index_mcp) # Mid
            #
            # #    index_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y]
            #     c = np.array(index_tip) # End
            #     radians = np.arctan2(c[1] - b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
            #     angle_index_tip = np.abs(radians*180.0/np.pi)
            #     if angle_index_tip > 180.0:
            #         angle_index_tip = 360-angle_index_tip
            #
            #     print(angle_index_tip)
            #
            #
            #     middle = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y * image_height]
            #     a = np.array(wrist) # First
            #     b = np.array(middle) # Mid
            #
            #     middle_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y * image_height]
            #     c = np.array(middle_tip) # End
            #     radians = np.arctan2(c[1] - b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
            #     angle_middle_tip = np.abs(radians*180.0/np.pi)
            #     if angle_middle_tip > 180.0:
            #         angle_middle_tip = 360-angle_middle_tip
            #
            #     print(angle_middle_tip)
            #
            #
            #     ring = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].y * image_height]
            #     a = np.array(wrist) # First
            #     b = np.array(ring) # Mid
            #
            #     ring_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y * image_height]
            #     c = np.array(ring_tip) # End
            #     radians = np.arctan2(c[1] - b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
            #     angle_ring_tip = np.abs(radians*180.0/np.pi)
            #     if angle_ring_tip > 180.0:
            #         angle_ring_tip = 360-angle_ring_tip
            #
            #     print(angle_ring_tip)
            #
            #
            #     pinky = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y * image_height]
            #     a = np.array(wrist) # First
            #     b = np.array(pinky) # Mid
            #
            #     pinky_tip = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y * image_height]
            #     c = np.array(pinky_tip) # End
            #     radians = np.arctan2(c[1] - b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
            #     angle_pinky_tip = np.abs(radians*180.0/np.pi)
            #     if angle_pinky_tip > 180.0:
            #         angle_pinky_tip = 360-angle_pinky_tip
            #
            #     print(angle_pinky_tip)
            #
            #     # Check for curved index finger
            #     index_pip = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].y * image_height]
            #     a = np.array(index_pip) # First
            #     b = np.array(index_dip) # Mid
            #     c = np.array(index_tip) # End
            #     radians = np.arctan2(c[1] - b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
            #     angle_index_curve = np.abs(radians*180.0/np.pi)
            #     if angle_index_curve > 180.0:
            #         angle_index_curve = 360-angle_index_curve
            #     print(angle_index_curve)
            #
            # #    # Distance thumb tip to index tip
            #     distance_ti = math.dist(thumb_tip,index_tip)
            #     print(distance_ti)
            #
            #     print(angle < 45)
            #     print((angle_index_tip - angle_middle_tip) < 45)
            #     print((angle_index_tip - angle_ring_tip) < 45)
            #     print((angle_index_tip - angle_pinky_tip) < 45)
            #     print(angle_index_curve < 90)
            #     print(distance_ti < 100)
            #
            #     grasp = 'None'
            #
            #
            #     middle_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y * image_height]
            #     ring_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].y * image_height]
            #     pinky_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y * image_height]
            #
            #     index_dip = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].y * image_height]
            #     middle_dip = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP].y * image_height]
            #     ring_dip = [hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_DIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_DIP].y * image_height]
            #     pinky_dip = [hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_DIP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_DIP].y * image_height]
            #
            #
            #
            #     a = np.array(index_mcp) # First
            #     b = np.array(index_dip) # Mid
            #     c = np.array(index_tip) # End
            #
            #     radians = np.arctan2(c[1] - b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
            #     angle_index = np.abs(radians*180.0/np.pi)
            #
            #     if angle_index > 180.0:
            #         angle_index = 360-angle_index
            #
            #     print(angle_index)
            #
            #
            #     a = np.array(middle_mcp) # First
            #     b = np.array(middle_dip) # Mid
            #     c = np.array(middle_tip) # End
            #
            #     radians = np.arctan2(c[1] - b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
            #     angle_middle = np.abs(radians*180.0/np.pi)
            #
            #     if angle_middle > 180.0:
            #         angle_middle = 360-angle_middle
            #
            #     print(angle_middle)
            #
            #
            #     a = np.array(ring_mcp) # First
            #     b = np.array(ring_dip) # Mid
            #     c = np.array(ring_tip) # End
            #
            #     radians = np.arctan2(c[1] - b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
            #     angle_ring = np.abs(radians*180.0/np.pi)
            #
            #     if angle_ring > 180.0:
            #         angle_ring = 360-angle_ring
            #
            #     print(angle_ring)
            #
            #
            #     a = np.array(pinky_mcp) # First
            #     b = np.array(pinky_dip) # Mid
            #     c = np.array(pinky_tip) # End
            #
            #     radians = np.arctan2(c[1] - b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
            #     angle_pinky = np.abs(radians*180.0/np.pi)
            #
            #     if angle_pinky > 180.0:
            #         angle_pinky = 360-angle_pinky
            #
            #     print(angle_pinky)
            #
            #     thumb_mcp = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].x * image_width,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].y * image_height]
            #
            #     distance_tt1 = math.dist(thumb_ip,thumb_mcp)
            #     distance_tt2 = math.dist(thumb_tip,thumb_ip)
            #     distance_tt = distance_tt1 + distance_tt2
            #
            #     print(distance_tt)
            #     print(distance_ti > distance_tt)
            #
            #     thumb_mcp_z = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].y,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP].z]
            #     thumb_ip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].x,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].y,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].z]
            #     thumb_tip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].z]
            #     thumb_cmc_z = [hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_CMC].x,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_CMC].y,hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_CMC].z]
            #
            #     distance_tt1_z = math.dist(thumb_ip_z,thumb_mcp_z)
            #     distance_tt2_z = math.dist(thumb_tip_z,thumb_ip_z)
            #     distance_tt3_z = math.dist(thumb_cmc_z,thumb_mcp_z)
            #     distance_tt_z = distance_tt1_z + distance_tt2_z
            #
            #     index_tip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].z]
            #     index_pip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].y,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].z]
            #     index_dip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].y,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].z]
            #     index_mcp_z = [hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y,hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].z]
            #
            #     distance_ti_z = math.dist(thumb_tip_z,index_tip_z)
            #
            #     middle_tip_z = [hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y,hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].z]
            #
            # #    distance_tm_z = math.dist(thumb_tip_z,middle_tip_z)
            #
            #     distance_ti1_z = math.dist(index_dip_z,index_pip_z)
            #     distance_ti2_z = math.dist(index_tip_z,index_dip_z)
            #     distance_ti3_z = math.dist(index_pip_z,index_mcp_z)
            #     distance_ti_z2 = distance_ti1_z + distance_ti2_z + distance_ti3_z
            #
            #     print(distance_ti_z)
            #     print(distance_ti_z2)
            #     print(distance_ti_z > distance_ti_z2)
            #
            #
            # #    if (angle_index_tip - angle_middle_tip) < 45 and (angle_index_tip - angle_ring_tip) > 45 and(angle_index_tip - angle_pinky_tip) > 45 and distance_ti > 250:
            #     if (angle_index - angle_middle) < 45 and (angle_index - angle_ring) > 45 and(angle_index - angle_pinky) > 45:
            # #        print("Person wants to grap the apple")
            #         grasp = 'Tripod'
            #         send_image('/home/rrl/ros_ws_baxter/Tripod.png')
            #     elif (angle_index_tip - angle_middle_tip) < 45 and (angle_index_tip - angle_ring_tip) < 45 and(angle_index_tip - angle_pinky_tip) > 45:
            # #        print("Person wants to grap the apple")
            #         grasp = 'Sphere 4 Finger'
            #         send_image('/home/rrl/ros_ws_baxter/Sphere4Finger.png')
            #     elif (angle_index_tip - angle_middle_tip) < 45 and (angle_index_tip - angle_ring_tip) < 45 and(angle_index_tip - angle_pinky_tip) < 45 and distance_ti_z > distance_ti_z2:
            # #        print("Person wants to grap the apple")
            #         grasp = 'Large Diameter'
            #         send_image('/home/rrl/ros_ws_baxter/LargeDiameter.png')
            # #    elif (angle_index_tip - angle_middle_tip) < 45 and (angle_index_tip - angle_ring_tip) < 45 and(angle_index_tip - angle_pinky_tip) < 45 and angle_index_curve > 90 and distance_ti < distance_tt:
            #     elif (angle_index_tip - angle_middle_tip) < 45 and (angle_index_tip - angle_ring_tip) < 45 and(angle_index_tip - angle_pinky_tip) < 45 and angle_index_curve > 135 and distance_ti_z < distance_ti_z2:
            # #        print("Person wants to grap the apple")
            #         grasp = 'Parallel Extension'
            #         send_image('/home/rrl/ros_ws_baxter/ParallelExtension.png')
            # #    elif (angle_index_tip - angle_middle_tip) < 45 and (angle_index_tip - angle_ring_tip) < 45 and(angle_index_tip - angle_pinky_tip) < 45 and angle_index_curve < 90 and distance_ti < distance_tt:
            #     elif (angle_index_tip - angle_middle_tip) < 45 and (angle_index_tip - angle_ring_tip) < 45 and(angle_index_tip - angle_pinky_tip) < 45 and angle_index_curve < 135 and distance_ti_z < distance_ti_z2:
            # #        print("Person wants to grap the apple")
            #         grasp = 'Extension Type'
            #         send_image('/home/rrl/ros_ws_baxter/ExtensionType.png')
            #     else:
            #         send_image('/home/rrl/ros_ws_baxter/HumanDetected.png')

            #     if (angle_index_tip - angle_middle_tip) < 45 and (angle_index_tip - angle_ring_tip) > 45 and(angle_index_tip - angle_pinky_tip) > 45 and distance_ti > 250:
            # #        print("Person wants to grap the apple")
            #         grasp = 'Tripod'
            #         send_image('/home/rrl/ros_ws_baxter/Tripod.png')
            #     elif (angle_index_tip - angle_middle_tip) < 45 and (angle_index_tip - angle_ring_tip) < 45 and(angle_index_tip - angle_pinky_tip) > 45 and distance_ti > 250:
            # #        print("Person wants to grap the apple")
            #         grasp = 'Sphere 4 Finger'
            #         send_image('/home/rrl/ros_ws_baxter/Sphere4Finger.png')
            #     elif (angle_index_tip - angle_middle_tip) < 45 and (angle_index_tip - angle_ring_tip) < 45 and(angle_index_tip - angle_pinky_tip) < 45 and distance_ti > 250:
            # #        print("Person wants to grap the apple")
            #         grasp = 'Large Diameter'
            #         send_image('/home/rrl/ros_ws_baxter/LargeDiameter.png')
            #     elif (angle_index_tip - angle_middle_tip) < 45 and (angle_index_tip - angle_ring_tip) < 45 and(angle_index_tip - angle_pinky_tip) < 45 and angle_index_curve > 90 and distance_ti < 250:
            # #        print("Person wants to grap the apple")
            #         grasp = 'Parallel Extension'
            #         send_image('/home/rrl/ros_ws_baxter/ParallelExtension.png')
            #     elif (angle_index_tip - angle_middle_tip) < 45 and (angle_index_tip - angle_ring_tip) < 45 and(angle_index_tip - angle_pinky_tip) < 45 and angle_index_curve < 90 and distance_ti < 250:
            # #        print("Person wants to grap the apple")
            #         grasp = 'Extension Type'
            #         send_image('/home/rrl/ros_ws_baxter/ExtensionType.png')


                # # Center coordinates
                # center_coordinates = (int(index[0]*image_width), int(index[1]*image_height))
                # # Radius of circle
                # radius = 20
                # # Blue color in BGR
                # color = (255, 0, 0)
                # # Line thickness of 2 px
                # thickness = 10
                # # Using cv2.circle() method
                # # Draw a circle with blue line borders of thickness of 2 px
                # image = cv2.circle(image, center_coordinates, radius, color, thickness)
                #
                # center_coordinates = (int(index_tip[0]*image_width), int(index_tip[1]*image_height))
                # # Radius of circle
                # radius = 20
                # # Blue color in BGR
                # color = (255, 0, 0)
                # # Line thickness of 2 px
                # thickness = 10
                # # Using cv2.circle() method
                # # Draw a circle with blue line borders of thickness of 2 px
                # image = cv2.circle(image, center_coordinates, radius, color, thickness)
                #
                # center_coordinates = (int(thumb_tip[0]*image_width), int(thumb_tip[1]*image_height))
                # # Radius of circle
                # radius = 20
                # # Blue color in BGR
                # color = (255, 0, 0)
                # # Line thickness of 2 px
                # thickness = 10
                # # Using cv2.circle() method
                # # Draw a circle with blue line borders of thickness of 2 px
                # image = cv2.circle(image, center_coordinates, radius, color, thickness)

                # font
                font = cv2.FONT_HERSHEY_SIMPLEX
                # org
                org = (50, 50)
                # fontScale
                fontScale = 1
                # Blue color in BGR
                color = (255, 0, 0)
                # Line thickness of 2 px
                thickness = 2
                image = cv2.putText(image, grasp, org, font,
                                   fontScale, color, thickness, cv2.LINE_AA)
                # cv2.imshow('MediaPipe Hands', image)
                # cv2.waitKey(0)





        # print("time.time")
        # print(timer())

        global idx
        idx = idx + 1
        # print('idx')
        # print(idx)
        cv2.imwrite('/home/rrl/ros_ws_baxter/images/annotated_image' + str(idx) + '.png', image)
        # cv2.imshow('MediaPipe Hands', image)

        # send_image('/home/rrl/ros_ws_baxter/HumanDetected.png')



    idx_yoloperson = idx_yoloperson + 1


def image_callback_run_yolo_once(ros_img):
    # Convert received image message to ros_numpy image
    np_img = ros_numpy.numpify(ros_img)
    os.chdir("../yolov7")
    np_img2 = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)

    # np_img2 = np_img2[int(500):int(800), int(380):int(780)]

    matplotlib.image.imsave('name.png', np_img2)

    if os.path.exists("/home/rrl/ros_ws_baxter/yolo/yolo/labels/name.txt"):
        os.remove("/home/rrl/ros_ws_baxter/yolo/yolo/labels/name.txt")
    # https://rockyshikoku.medium.com/training-yolov7-b7b7e4606d22
    # class x_center y_center width height
    # Box coordinates are in xywh format normalized to 0 to 1.
    # os.system("python3 detect.py --weights yolov7.pt --conf 0.25 --img-size 640 --source name.png --name yolo --project ../ros_ws_baxter/yolo --exist-ok --save-txt --save-conf")
    os.system("python3.8 detect.py --weights yolov7.pt --conf 0.2 --img-size 640 --source name.png --name yolo --project ../ros_ws_baxter/yolo --exist-ok --save-txt")
    os.chdir("../ros_ws_baxter")
    find_object_positions(np_img)
    sub.unregister()


def find_object_positions(np_img):

    data = np.loadtxt("/home/rrl/ros_ws_baxter/yolo/yolo/labels/name.txt", dtype=float)
    # print("data find_object_positions")
    # print(data)
    # print(type(data))

    lst2 = [int(item[0]) for item in data]
    # print(lst2)

    # bowl: 45
    # book: 73
    # sports ball: 32

    # # scissors:
    #
    # index = lst2.index(76)
    # print(index)
    # print(data[index])
    #
    # global x_center_scissors, y_center_scissors
    #
    # x_center_scissors = data[index][1]*1280
    # y_center_scissors = data[index][2]*800
    #
    # print("x_center_scissors original")
    # print(x_center_scissors)
    # print(y_center_scissors)
    #
    # image = cv2.circle(np_img, ( int(x_center_scissors) , int(y_center_scissors)   ), radius=0, color=(0, 0, 255), thickness=5)
    # #
    # # find minimum y of objects
    # height_scissors = data[index][3]*1280
    # y_min_scissors = y_center_scissors + height_scissors/2
    # # image = cv2.circle(np_img, ( int(x_center) , int(y_min_scissors)   ), radius=0, color=(255, 0, 0), thickness=10)

    y_min_ball = 1000
    y_min_bowl = 1000
    y_min_book = 1000

    # bowl: (or frisbee)

    if (45 in lst2) or (29 in lst2):

        # index = lst2.index(45)
        # print(index)
        # print(data[index])


        if (45 in lst2):

            index = lst2.index(45)
            # print('cup')
            # print(index)
            # print(data[index])

        if (29 in lst2):

            index = lst2.index(29)
            # print('ball')
            # print(index)
            # print(data[index])


        global x_center_bowl, y_center_bowl, width_bowl

        x_center_bowl = data[index][1]*1280
        y_center_bowl = data[index][2]*800

        width_bowl = data[index][3]*1280

        # print("x_center_bowl original")
        # print(x_center_bowl)

        image = cv2.circle(np_img, ( int(x_center_bowl) , int(y_center_bowl)   ), radius=0, color=(0, 0, 255), thickness=5)

        # find minimum y of objects
        height_bowl = data[index][3]*1280
        y_min_bowl = y_center_bowl + height_bowl/2

    # book:

    if (73 in lst2):

        index = lst2.index(73)
        # print(index)
        # print(data[index])

        global x_center_book, y_center_book, width_book

        x_center_book = data[index][1]*1280
        y_center_book = data[index][2]*800

        width_book = data[index][3]*1280

        # print("x_center_book original")
        # print(x_center_book)
        # print(y_center_book)
        # print(width_book)

        image = cv2.circle(np_img, ( int(x_center_book) , int(y_center_book)   ), radius=0, color=(0, 0, 255), thickness=5)

        # find minimum y of objects
        height_book = data[index][3]*1280
        y_min_book = y_center_book + height_book/2


    # ball:

    # if (47 in lst2):
    #     index = lst2.index(47)
    #     print(index)
    #     print(data[index])
    # else:
    #     index = lst2.index(32)
    #     print(index)
    #     print(data[index])

    if (32 in lst2) or (41 in lst2):

        if (41 in lst2):

            index = lst2.index(41)
            # print('cup')
            # print(index)
            # print(data[index])

        if (32 in lst2):

            index = lst2.index(32)
            # print('ball')
            # print(index)
            # print(data[index])

        global x_center_ball, y_center_ball

        x_center_ball = data[index][1]*1280
        y_center_ball = data[index][2]*800

        # print("x_center_ball original")
        # print(x_center_ball)
        # print(y_center_ball)

        image = cv2.circle(np_img, ( int(x_center_ball) , int(y_center_ball)   ), radius=0, color=(0, 0, 255), thickness=5)

        # find minimum y of objects
        height_ball = data[index][3]*1280
        y_min_ball = y_center_ball + height_ball/2

    # Modify

    global y_min
    y_min = min(y_min_bowl, y_min_book, y_min_ball)
    # y_min = min(y_min_scissors, y_min_apple)
    # image = cv2.circle(image, ( int(x_center) , int(y_min)   ), radius=0, color=(255, 0, 0), thickness=5)

    # print("y_min_imageyolo")
    # print(y_min)

    cv2.imwrite('/home/rrl/ros_ws_baxter/ImageYOLO.png', image)

    image = cv2.imread('/home/rrl/ros_ws_baxter/ImageYOLO.png')
    image_height, image_width, _ = image.shape
    # print(image_width)
    # print(image_height)


# def prnt_cu():
#     print("Cube")


if __name__ == '__main__':
    print("============ Starting tutorial setup")

    moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('move_group_python_interface_tutorial',
    #               anonymous=True)

    rospy.init_node('Camera_Subscriber',anonymous=True) # Initialize ROS node

    # Run Yolo once at beginning since the objects are not moving
    sub = rospy.Subscriber('/cameras/head_camera/image', Image, image_callback_run_yolo_once)

    global idx, idx_yoloperson
    idx = 1
    idx_yoloperson = 1

    # global x_center_scissors, y_center_scissors
    # x_center_scissors = 0
    # y_center_scissors = 0

    global x_center_book, y_center_book
    x_center_book = 0
    y_center_book = 0

    global x_center_bowl, y_center_bowl
    x_center_bowl = 0
    y_center_bowl = 0

    global x_center_ball, y_center_ball
    x_center_ball = 0
    y_center_ball = 0

    # Choose which object the robotarm should move to randomly
    global object_select
    object_select = random.choice( ["bowl", "book", "ball"] )
    object_select = "bowl"
    print("object_select")
    print(object_select)

    # # Choose which robotarm should move randomly
    # global robotarm
    # # mylist = ["left", "right"]
    # robotarm = random.choice( ["left", "right"] )
    # print("robotarm")
    # print(robotarm)
    #
    rs = baxter_interface.RobotEnable()
    rs.enable()
    global robotarm
    # if robotarm == 'left':
    if object_select == 'bowl' or object_select == 'book':
        robotarm = 'left'
        pickplace = Pick_and_Place('left_arm','left')
        pickplace.input_pos("/home/rrl/ros_ws_baxter/position_leftarm.txt")
    else:
        robotarm = 'right'
        pickplace = Pick_and_Place('right_arm','right')
        pickplace.input_pos("/home/rrl/ros_ws_baxter/position_rightarm.txt")

    # pickplace.pick_place_implementation()
    #
    # # Subscribe to head_camera image topic
    # rospy.Subscriber('/cameras/head_camera/image', Image, image_callback)

    # print('before')
    # rospy.sleep(10)

    # creating multiple processes
    # proc1 = multiprocessing.Process(target=prnt_cu)
    # proc1 = multiprocess.Process(target=pickplace.pick_place_implementation)

    # proc1 = multiprocessing.Process(target=pickplace.pick_place_implementation)
    # proc2 = multiprocessing.Process(target=rospy.Subscriber, args=('/cameras/head_camera/image', Image, image_callback,))

    global exit_flag
    exit_flag = False

    proc1 = threading.Thread(target=pickplace.pick_place_implementation)
    # proc1 = threading.Thread(target=pickplace.pick_place_implementation, args =(lambda : exit_flag, ))
    proc2 = threading.Thread(target=rospy.Subscriber, args=('/cameras/head_camera/image', Image, image_callback))

    # print('after')
    # proc2 = multiprocessing.Process(target=rospy.Subscriber, args=('/cameras/head_camera/image', Image, image_callback))
    # proc2 = multiprocessing.Process(target=prnt_cu)

    # Initiating process 1
    proc1.start()
    proc2.start()

    # Initiating process 2
    # proc2.start()
    # Waiting until proc1 finishes
    # proc1.join()
    # Waiting until proc2 finishes
    # proc2.join()

    rospy.spin() # sleep
    cv2.destroyAllWindows() # Destroy CV image window on shut_down

    send_image('/home/rrl/ros_ws_baxter/src/baxter_tools/share/images/researchsdk.png')

    print("END OF PROGRAM!!!")
