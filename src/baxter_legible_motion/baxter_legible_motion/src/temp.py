#!/usr/bin/env python3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty

import trajectory_msgs.msg

import math
import copy
import argparse

import baxter_interface

import itertools

from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import rospkg
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

import os
absolute_path = os.path.dirname(__file__)
sys.path.append(absolute_path)

from trajopt.solver.slsqp import SLSQPSolver
import trajopt.spacetime.spacetime
import trajopt.robot.builtinRobots
import trajopt.spacetime.builtinObjectives
import trajopt.spacetime.builtinConstraints

import tf2_ros
import tf2_geometry_msgs
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, rotation_matrix, concatenate_matrices, quaternion_from_matrix
from scipy.spatial.transform import Rotation
import numpy as np
from sklearn import preprocessing

from urdf_parser_py.urdf import URDF
sys.path.append(os.path.join(absolute_path,'hrl-kdl','hrl_geom','src'))
sys.path.append(os.path.join(absolute_path,'hrl-kdl','pykdl_utils','src'))
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
import random

from visualization_msgs.msg import Marker

import matplotlib.pyplot as plt
import numpy as np
import math
import random
import sys

objects = []

class ExampleMoveItTrajectories(object):

    """ExampleMoveItTrajectories"""
    def __init__(self):

        # Initialize the node
        joint_state_topic = ['joint_states:=/robot/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('example_move_it_trajectories', anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)

        self.roll_gripper = 0.0
        self.pitch_gripper = 0.0
        self.yaw_gripper = 0.0
        self.artag_data = PoseStamped()
        self.object_transform = geometry_msgs.msg.TransformStamped()
        self.transform = geometry_msgs.msg.TransformStamped()
        self.counter = 0
        self.rotation_cup_data = Quaternion()

        # Create the MoveItInterface necessary objects
        group_name = "right_arm"
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

def main(execute_type):
    example = ExampleMoveItTrajectories()

    ##### initial setup for the whole algo #####

    # get target and obstacle positions
    group = example.group

    group.set_max_velocity_scaling_factor(0.7)

    joint_goal_home = group.get_current_joint_values()

    joint_goal_home[0] = -0.6323835798057618
    joint_goal_home[1] = -1.294296289778318
    joint_goal_home[2] = -0.1062281695610649
    joint_goal_home[3] = 2.524932376859391
    joint_goal_home[4] = 0.07593204900032798
    joint_goal_home[5] =  0.3566505331833587
    joint_goal_home[6] = 1.6869953714769839

    joint_goal_handoff = group.get_current_joint_values()

    joint_goal_handoff[0] = 0.3589515043651868
    joint_goal_handoff[1] = -0.3359417925469056
    joint_goal_handoff[2] = 0.5656554155327463
    joint_goal_handoff[3] = 1.2735875491418647
    joint_goal_handoff[4] =-0.3014272248194838
    joint_goal_handoff[5] =  -0.6975777632908919
    joint_goal_handoff[6] = 2.886951842800349

    if execute_type == 'home':
        rospy.loginfo('hello')

        group.go(joint_goal_home, wait=True)

    elif execute_type == 'handoff':
        rospy.loginfo('handoff')

        group.go(joint_goal_handoff, wait=True)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        prog='ProgramName',
        description='What the program does',
        epilog='Text at the bottom of help')

    parser.add_argument('-x', '--execute', default='home')

    args = parser.parse_args()

    main(args.execute)
