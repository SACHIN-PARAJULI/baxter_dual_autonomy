#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run :
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty

# trajectory_msgs/JointTrajectoryPoint.msg
import trajectory_msgs.msg

import math
import copy

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

sys.path.append("/home/rrl/ros_ws_baxter_collision_detection/src/pick_and_place_baxter_moveit/src")
# sys.path.append("/home/melanie/kinova_ws/src/ros_kortex/kortex_examples/src")
from trajopt.solver.slsqp import SLSQPSolver
from trajopt.spacetime.spacetime import Spacetime
from trajopt.robot.builtinRobots import Baxter
from trajopt.spacetime.builtinObjectives import StateVelocity, AlignAxis, LegibleS, LegibleG, PotentialFunction
from trajopt.spacetime.builtinConstraints import Nail, alignAxisGT, AboveFloor, pointDistance
from trajopt.utilities.fileOperations import writeTrajectoryFile

# from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
import tf2_ros
import tf2_geometry_msgs
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, rotation_matrix, concatenate_matrices
from scipy.spatial.transform import Rotation
import numpy as np
from sklearn import preprocessing


def load_gazebo_models(table_pose=Pose(position=Point(x=0.7, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       # power_drill_pose=Pose(position=Point(x=0.5, y=0.3, z=0.5)),
                       # power_drill_reference_frame="world",
                       # wood_block_pose=Pose(position=Point(x=0.4, y=-0.25, z=0.3)),
                       # wood_block_reference_frame="world",
                       # scissors_pose=Pose(position=Point(x=0.55, y=-0.1, z=0.35)),
                       scissors_pose=Pose(position=Point(x=0.75, y=-0.1, z=0.78)),
                       scissors_reference_frame="world",
                       # large_marker_pose=Pose(position=Point(x=0.4, y=0, z=0.4)),
                       # large_marker_reference_frame="world",
                       # adjustable_wrench_pose=Pose(position=Point(x=0.6, y=0.25, z=0.35)),
                       # adjustable_wrench_reference_frame="world",
                       # flat_screwdriver_pose=Pose(position=Point(x=0.6, y=0, z=0.35)),
                       # flat_screwdriver_reference_frame="world",
                       # hammer_pose=Pose(position=Point(x=0.5, y=0, z=0.4)),
                       # hammer_reference_frame="world",
                       # clock_pose=Pose(position=Point(x=0.45, y=0, z=0.4)),
                       # clock_reference_frame="world",
                       # bowl_pose=Pose(position=Point(x=0.55, y=0.2, z=0.4)),
                       # bowl_reference_frame="world",
                       # mug_pose=Pose(position=Point(x=0.57, y=0, z=0.4)),
                       # mug_reference_frame="world",
                       # tennis_ball_pose=Pose(position=Point(x=0.6, y=0.15, z=0.35)),
                       clock_pose=Pose(position=Point(x=0.65, y=0, z=0.78)),
                       clock_reference_frame="world",
                       bowl_pose=Pose(position=Point(x=0.75, y=0.2, z=0.78)),
                       bowl_reference_frame="world",
                       mug_pose=Pose(position=Point(x=0.77, y=0, z=0.78)),
                       mug_reference_frame="world",
                       tennis_ball_pose=Pose(position=Point(x=0.8, y=0.15, z=0.78)),
                       tennis_ball_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_gazebo')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # # Load Block URDF
    # power_drill_xml = ''
    # with open (model_path + "035_power_drill/power_drill.sdf", "r") as power_drill_file:
    #     power_drill_xml=power_drill_file.read().replace('\n', '')
    # # Spawn Table SDF
    # rospy.wait_for_service('/gazebo/spawn_sdf_model')

    # # Load Block URDF
    # wood_block_xml = ''
    # with open (model_path + "036_wood_block/wood_block.sdf", "r") as wood_block_file:
    #     wood_block_xml=wood_block_file.read().replace('\n', '')
    # # Spawn Table SDF
    # rospy.wait_for_service('/gazebo/spawn_sdf_model')

    # Load scissors URDF
    scissors_xml = ''
    with open (model_path + "037_scissors/scissors.sdf", "r") as scissors_file:
        scissors_xml=scissors_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    # # Load large_marker URDF
    # large_marker_xml = ''
    # with open (model_path + "040_large_marker/large_marker.sdf", "r") as large_marker_file:
    #     large_marker_xml=large_marker_file.read().replace('\n', '')
    # # Spawn Table SDF
    # rospy.wait_for_service('/gazebo/spawn_sdf_model')
    #
    # # Load adjustable_wrench URDF
    # adjustable_wrench_xml = ''
    # with open (model_path + "042_adjustable_wrench/adjustable_wrench.sdf", "r") as adjustable_wrench_file:
    #     adjustable_wrench_xml=adjustable_wrench_file.read().replace('\n', '')
    # # Spawn Table SDF
    # rospy.wait_for_service('/gazebo/spawn_sdf_model')
    #
    # # Load flat_screwdriver URDF
    # flat_screwdriver_xml = ''
    # with open (model_path + "044_flat_screwdriver/flat_screwdriver.sdf", "r") as flat_screwdriver_file:
    #     flat_screwdriver_xml=flat_screwdriver_file.read().replace('\n', '')
    # # Spawn Table SDF
    # rospy.wait_for_service('/gazebo/spawn_sdf_model')
    #
    # # Load hammer URDF
    # hammer_xml = ''
    # with open (model_path + "048_hammer/hammer.sdf", "r") as hammer_file:
    #     hammer_xml=hammer_file.read().replace('\n', '')
    # # Spawn Table SDF
    # rospy.wait_for_service('/gazebo/spawn_sdf_model')

    # Load clock URDF
    clock_xml = ''
    with open (model_path + "076_timer/timer.sdf", "r") as clock_file:
        clock_xml=clock_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    # Load bowl URDF
    bowl_xml = ''
    with open (model_path + "024_bowl/bowl.sdf", "r") as bowl_file:
        bowl_xml=bowl_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    # Load mug URDF
    mug_xml = ''
    with open (model_path + "025_mug/mug.sdf", "r") as mug_file:
    # with open (model_path + "mug_artag/mug_artag.sdf", "r") as mug_file:
        mug_xml=mug_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    # Load tennis_ball URDF
    tennis_ball_xml = ''
    with open (model_path + "056_tennis_ball/tennis_ball.sdf", "r") as tennis_ball_file:
        tennis_ball_xml=tennis_ball_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # # Spawn Block URDF
    # rospy.wait_for_service('/gazebo/spawn_urdf_model')
    # try:
    #     spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #     resp_sdf = spawn_sdf("power_drill", power_drill_xml, "/",
    #                          power_drill_pose, power_drill_reference_frame)
    # except rospy.ServiceException as e:
    #     rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # # Spawn Block URDF
    # rospy.wait_for_service('/gazebo/spawn_urdf_model')
    # try:
    #     spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #     resp_sdf = spawn_sdf("wood_block", wood_block_xml, "/",
    #                          wood_block_pose, wood_block_reference_frame)
    # except rospy.ServiceException as e:
    #     rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn scissors URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("scissors", scissors_xml, "/",
                             scissors_pose, scissors_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # # Spawn large_marker URDF
    # rospy.wait_for_service('/gazebo/spawn_urdf_model')
    # try:
    #     spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #     resp_sdf = spawn_sdf("large_marker", large_marker_xml, "/",
    #                          large_marker_pose, large_marker_reference_frame)
    # except rospy.ServiceException as e:
    #     rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    #
    # # Spawn adjustable_wrench URDF
    # rospy.wait_for_service('/gazebo/spawn_urdf_model')
    # try:
    #     spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #     resp_sdf = spawn_sdf("adjustable_wrench", adjustable_wrench_xml, "/",
    #                          adjustable_wrench_pose, adjustable_wrench_reference_frame)
    # except rospy.ServiceException as e:
    #     rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    #
    # # Spawn flat_screwdriver URDF
    # rospy.wait_for_service('/gazebo/spawn_urdf_model')
    # try:
    #     spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #     resp_sdf = spawn_sdf("flat_screwdriver", flat_screwdriver_xml, "/",
    #                          flat_screwdriver_pose, flat_screwdriver_reference_frame)
    # except rospy.ServiceException as e:
    #     rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    #
    # # Spawn hammer URDF
    # rospy.wait_for_service('/gazebo/spawn_urdf_model')
    # try:
    #     spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #     resp_sdf = spawn_sdf("hammer", hammer_xml, "/",
    #                          hammer_pose, hammer_reference_frame)
    # except rospy.ServiceException as e:
    #     rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn clock URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("clock", clock_xml, "/",
                             clock_pose, clock_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn bowl URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("bowl", bowl_xml, "/",
                             bowl_pose, bowl_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn mug URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("mug", mug_xml, "/",
                             mug_pose, mug_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn tennis_ball URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("tennis_ball", tennis_ball_xml, "/",
                             tennis_ball_pose, tennis_ball_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        # resp_delete = delete_model("power_drill")
        # resp_delete = delete_model("wood_block")

        resp_delete = delete_model("scissors")

        # resp_delete = delete_model("large_marker")
        # resp_delete = delete_model("adjustable_wrench")
        # resp_delete = delete_model("flat_screwdriver")
        # resp_delete = delete_model("hammer")

        resp_delete = delete_model("clock")
        resp_delete = delete_model("bowl")
        resp_delete = delete_model("mug")
        resp_delete = delete_model("tennis_ball")
    except rospy.ServiceException as e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


class ExampleMoveItTrajectories(object):

    """ExampleMoveItTrajectories"""
    def __init__(self):

        # Initialize the node
        # super(ExampleMoveItTrajectories, self).__init__()
        # moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('example_move_it_trajectories')

        joint_state_topic = ['joint_states:=/robot/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('example_move_it_trajectories', anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)

        # rostopic info /my_gen3/base_feedback
        # Type: kortex_driver/BaseCyclic_Feedback
        self.roll_gripper = 0.0
        self.pitch_gripper = 0.0
        self.yaw_gripper = 0.0
        self.artag_data = PoseStamped()
        self.object_transform = geometry_msgs.msg.TransformStamped()
        self.transform = geometry_msgs.msg.TransformStamped()
        self.counter = 0
        self.rotation_cup_data = Quaternion()

        # try:
        # self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
        # if self.is_gripper_present:
        #     gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        #     self.gripper_joint_name = gripper_joint_names[0]
        # else:
        #     self.gripper_joint_name = ""
        # self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

        # Create the MoveItInterface necessary objects
        group_name = "right_arm"
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        # self.robot = moveit_commander.RobotCommander("robot_description")
        # self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())

        # self.group = moveit_commander.MoveGroupCommander(group_name, ns=rospy.get_namespace())
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
        #
        # self.link_name = ''
        # self.link_pose = Pose()
        #
        # if self.is_gripper_present:
        #     gripper_group_name = "gripper"
        #     self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())
        #
        #     rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())

        self.gripper = baxter_interface.Gripper("right")
        self.gripper.calibrate()
        self.gripper.set_holding_force(100.0)

        # except Exception as e:
        #     print (e)
        #     self.is_init_success = False
        # else:
        #     self.is_init_success = True


    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group

        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        print(gripper_max_absolute_pos)
        print(gripper_min_absolute_pos)
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except:
            return False

    def moveit(self):
        group = self.group
        joint_positions = group.get_current_joint_values()

        # # move to home position
        # joint_positions[0] = 0
        # joint_positions[1] = 0
        # joint_positions[2] = pi
        # joint_positions[3] = -pi/4
        # joint_positions[4] = 0
        # joint_positions[5] = -pi/2
        # joint_positions[6] = -pi/2

       # 'left':  [-0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50],
       # 'right':  [0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50]

        # move to home position
        joint_positions[0] = 0.08
        joint_positions[1] = -1.0
        joint_positions[2] = 1.19
        joint_positions[3] = 1.94
        joint_positions[4] = -0.67
        joint_positions[5] = 1.03
        joint_positions[6] = 0.50

        group.set_joint_value_target(joint_positions)
        # Plan and execute in one command
        group.go(wait=True)


    def artag_callback(self, data):

        if self.counter == 0:

            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)
            try:
                self.transform = tfBuffer.lookup_transform('base_link', 'camera_link_new', rospy.Time.now(), rospy.Duration(1.0))
                print('self.transform')
                print(self.transform)

                np.set_printoptions(suppress=True, precision=6)

                for marker in data.markers:
                    print('marker')
                    print(marker)
                    self.artag_data.pose = marker.pose.pose
                    print(self.artag_data.pose.position)

                    rot_quat_transform = np.array([self.transform.transform.rotation.x, self.transform.transform.rotation.y, self.transform.transform.rotation.z, self.transform.transform.rotation.w]) * np.array([ 0.5, -0.5, 0.5, -0.5 ])
                    # print(rot_quat_transform)
                    rot_from_quat = Rotation.from_quat(rot_quat_transform)
                    rot_rotation = rot_from_quat.as_matrix()
                    # print(rot_rotation)
                    rot_marker_position_camera = rot_rotation @ [self.artag_data.pose.position.x, self.artag_data.pose.position.y, self.artag_data.pose.position.z]
                    # print(rot_marker_position_camera)
                    rot_marker_position = [self.transform.transform.translation.x, self.transform.transform.translation.y, self.transform.transform.translation.z] - rot_marker_position_camera
                    # print('rot_marker_position')
                    # print(rot_marker_position)
                    # get camera pose rotation
                    rot = Rotation.from_quat([self.artag_data.pose.orientation.x, self.artag_data.pose.orientation.y, self.artag_data.pose.orientation.z, self.artag_data.pose.orientation.w])
                    # print(rot)

                    rot_camera_pose = rot.as_matrix()
                    rot_mod = np.array([ [0,1,0], [0,0,1], [1,0,0] ])
                    rot_camera_pose_mod = rot_mod @ rot_camera_pose
                    # print(rot_camera_pose_mod)
                    rot = Rotation.from_matrix(rot_camera_pose_mod)
                    rot_camera_pose_mod = rot.as_quat()
                    rot_marker_rotation_quat_pre = rot_camera_pose_mod * rot_quat_transform
                    rot = Rotation.from_quat(rot_marker_rotation_quat_pre)
                    rot_camera_pose = rot.as_matrix()
                    rot_mod = np.array([ [0,1,0], [0,0,1], [1,0,0] ])
                    rot_camera_pose_mod = rot_mod @ rot_camera_pose
                    rot_mod = np.array([ [0,1,0], [-1,0,0], [0,0,1] ])
                    rot_camera_pose_mod = rot_mod @ rot_camera_pose_mod
                    rot = Rotation.from_matrix(rot_camera_pose_mod)
                    rot_marker_rotation_quat = rot.as_quat()

                    br = tf2_ros.TransformBroadcaster()
                    # t = geometry_msgs.msg.TransformStamped()
                    self.object_transform.header.stamp = rospy.Time.now()
                    self.object_transform.header.frame_id = "base_link"
                    self.object_transform.child_frame_id = "turtlename"
                    self.object_transform.transform.translation.x = rot_marker_position[0]
                    self.object_transform.transform.translation.y = 0.07 - rot_marker_position[1]
                    self.object_transform.transform.translation.z = rot_marker_position[2]
                    self.object_transform.transform.rotation.x = rot_marker_rotation_quat[0]
                    self.object_transform.transform.rotation.y = rot_marker_rotation_quat[1]
                    self.object_transform.transform.rotation.z = rot_marker_rotation_quat[2]
                    self.object_transform.transform.rotation.w = rot_marker_rotation_quat[3]
                    br.sendTransform(self.object_transform)
                    print('self.object_transform')
                    print(self.object_transform)

                    rot_from_quat = Rotation.from_quat(rot_marker_rotation_quat)
                    rot_rotation = rot_from_quat.as_euler('xyz', degrees=False)
                    self.roll_gripper = rot_rotation[0]
                    self.pitch_gripper = rot_rotation[1]
                    self.yaw_gripper = rot_rotation[2]

                    self.counter = 1

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print('continue')

        rospy.sleep(1)

    def rotation_cup_callback(self, data):
        self.rotation_cup_data = data
        print('rotation_cup_data')
        print(data)

    def trajectory_legible_test(self, position_joints_start, position_joints_target):
        # Create Robot
        # ready2 = [0, 0, pi, -pi/2, 0, 0, pi/2]
        # robot = KinovaGen3()
        # robot.default = ready2

        # robot = KinovaGen3()
        robot = Baxter()
        robot.default = position_joints_start

        # Create Spacetime Problem
        numFrames = 15
        # numFrames = 5
         # mug_pose=Pose(position=Point(x=0.77, y=0, z=0.78))
        # targetPosition = (0.5, 0.0, 0.3)
        targetPosition = (0.77, 0.0, 0.8)
        targetState = list(position_joints_target)
        print('targetState')
        print(targetState)

        st = Spacetime(robot, numFrames)
        st.defaultState[numFrames - 1] = targetState
        st.excludeKeys = [0]
        st.excludeKeys.append(numFrames - 1)

        # Setup Objective (minimum joint velocity)
        minJointVelocityTerm = StateVelocity()
        minJointVelocityWeight = 100.0
        st.addPointObjective(minJointVelocityTerm, minJointVelocityWeight) # add to objectives

        # Setup Objective (align end-effector axis upwards)
        robotEndEffectorUpVector = (1, 0, 0)
        robotEndEffectorUpAxis = 1
        robotEndEffectorPoint = -1 # last point on robot

        alignAxisTerm = AlignAxis(robotEndEffectorPoint, robotEndEffectorUpAxis, robotEndEffectorUpVector)
        minJointVelocityWeight = 25.0
        st.addPointObjective(alignAxisTerm, minJointVelocityWeight) # add to objectives

        # Setup Constraints
           # scissors_pose=Pose(position=Point(x=0.55, y=-0.1, z=0.35)),
           # clock_pose=Pose(position=Point(x=0.45, y=0, z=0.4)),
           # bowl_pose=Pose(position=Point(x=0.55, y=0.2, z=0.4)),
           # tennis_ball_pose=Pose(position=Point(x=0.6, y=0.15, z=0.35))
        pointID = -1 # the point we are interested in
        # st.c1 = pointDistance(pointID,0.1,0.55,-0.1,0.35,True) # obstacle
        # st.c2 = pointDistance(pointID,0.1,0.45,0,0.4,True)  # obstacle
        # st.c3 = pointDistance(pointID,0.1,0.55,0.2,0.4,True)  # obstacle
        # st.c4 = pointDistance(pointID,0.1,0.6,0.15,0.35,True)  # obstacle

        # Nail end position at target location
        # st.addAllTimeConstraint(st.c1)
        # st.addAllTimeConstraint(st.c2)
        # st.addAllTimeConstraint(st.c3)
        # st.addAllTimeConstraint(st.c4)
        # st.addConstraint(2, AboveFloor(3, 0.3))
        # st.addConstraint(7, AboveFloor(3, 0.3))
        st.addConstraint(numFrames - 1, Nail(robotEndEffectorPoint, targetPosition, False))
        st.addConstraint(numFrames - 1, alignAxisGT(robotEndEffectorPoint, robotEndEffectorUpAxis, robotEndEffectorUpVector))

        # # st.addPointObjective(SO.StateVelocity(), ptvel)
        # legw = 7.5
        # def closePointToLine(_eePoint, _eeVel, _eeAlign, _pathRatio):
        #     print('_eePoint')
        #     print(_eePoint)
        #     print('_pathRatio')
        #     print(_pathRatio)
        #     xCoord = _eePoint[0]
        #     if (xCoord > 5):
        #         xCoord = 5
        #     elif (xCoord < -4):
        #         xCoord = -4
        #     P = np.array([xCoord, 5, 0])
        #     print('P')
        #     print(P)
        #     return P
        # st.addPointObjective(LegibleG(pointID, targetPosition, numFrames, closePointToLine), legw)  # g vector term
        # st.addPointObjective(LegibleS(pointID, targetPosition, numFrames), legw / 10)  # s vector term

        # # Calculate legible state of the art
        # legw = 7.5
        # # take extrema of objects
        # def closePointToLine(_eePoint, _eeVel, _eeAlign, _pathRatio):
        #     # print('_eePoint')
        #     # print(_eePoint)
        #     # print('_pathRatio')
        #     # print(_pathRatio)
        #     xCoord = _eePoint[0]
        #     yCoord = _eePoint[1]
        #     if (xCoord > 0.6):
        #         xCoord = 0.6
        #     elif (xCoord < 0.45):
        #         xCoord = 0.45
        #     if (yCoord > 0.2):
        #         yCoord = 0.2
        #     # elif (yCoord < -0.1):
        #     #     yCoord = -0.1
        #     # if (xCoord > 5):
        #     #     xCoord = 5
        #     # elif (xCoord < -4):
        #     #     xCoord = -4
        #     # P = np.array([xCoord, 5, 0])
        #     P = np.array([xCoord, yCoord, 0.8])
        #     # P = np.array([xCoord, 0, 0.5])
        #     print('P')
        #     print(P)
        #     return P
        # st.addPointObjective(LegibleG(pointID, targetPosition, numFrames, closePointToLine), legw)  # g vector term
        # st.addPointObjective(LegibleS(pointID, targetPosition, numFrames), legw / 10)  # s vector term

        # # replace LegibleG and LegibleS with Potential Field Force for st.addPointObjective
        # def potentialFunctionCalc(_eePoint, _eeVel, _eeAlign, _pathRatio):
        #     # zeta: attractive potential gain
        #     zeta = 0.1
        #     # ro_g: relative Cartesian distance between the end effector of the arm and the target
        #     # pose_goal = self.group.get_current_pose().pose
        #     # eePosition = (pose_goal.position.x, pose_goal.position.y, pose_goal.position.z)
        #     eePosition = (_eePoint[0], _eePoint[1], _eePoint[2])
        #     ro_g = math.dist(eePosition, targetPosition)
        #     print('ro_g')
        #     print(ro_g)
        #     U_potential_gradient_att = zeta * ro_g
        #     # k_rep: repulsive potential gain
        #     k_rep = 0.01
        #     # ro_0: max range of the repulsive potential field
        #     ro_0 = 0.075
        #     # ro_b: minimum distance between the robot's body and the obstacle/ measures the workspace distance between the robot and the obstacle
        #        # scissors_pose=Pose(position=Point(x=0.55, y=-0.1, z=0.35)),
        #        # clock_pose=Pose(position=Point(x=0.45, y=0, z=0.4)),
        #        # bowl_pose=Pose(position=Point(x=0.55, y=0.2, z=0.4)),
        #        # tennis_ball_pose=Pose(position=Point(x=0.6, y=0.15, z=0.35))
        #     obstaclePosition_1 = (0.75, -0.1, eePosition[2])
        #     ro_b_1 = math.dist(eePosition, obstaclePosition_1)
        #     if ro_b_1 <= ro_0:
        #         U_potential_gradient_rep_1 = k_rep * (1/ro_b_1 - 1/ro_0) * (1/(ro_b_1**2))
        #     else:
        #         U_potential_gradient_rep_1 = 0.0
        #     obstaclePosition_2 = (0.65, 0, eePosition[2])
        #     ro_b_2 = math.dist(eePosition, obstaclePosition_2)
        #     if ro_b_2 <= ro_0:
        #         U_potential_gradient_rep_2 = k_rep * (1/ro_b_2 - 1/ro_0) * (1/(ro_b_2**2))
        #     else:
        #         U_potential_gradient_rep_2 = 0.0
        #     obstaclePosition_3 = (0.75, 0.2, eePosition[2])
        #     ro_b_3 = math.dist(eePosition, obstaclePosition_3)
        #     if ro_b_3 <= ro_0:
        #         U_potential_gradient_rep_3 = k_rep * (1/ro_b_3 - 1/ro_0) * (1/(ro_b_3**2))
        #     else:
        #         U_potential_gradient_rep_3 = 0.0
        #     obstaclePosition_4 = (0.8, 0.15, eePosition[2])
        #     ro_b_4 = math.dist(eePosition, obstaclePosition_4)
        #     if ro_b_4 <= ro_0:
        #         U_potential_gradient_rep_4 = k_rep * (1/ro_b_4 - 1/ro_0) * (1/(ro_b_4**2))
        #     else:
        #         U_potential_gradient_rep_4 = 0.0
        #     U_potential_gradient_rep = U_potential_gradient_rep_1 + U_potential_gradient_rep_2 + U_potential_gradient_rep_3 + U_potential_gradient_rep_4
        #     # U_potential_gradient = U_potential_gradient_att + U_potential_gradient_rep
        #     U_potential_gradient = U_potential_gradient_rep
        #     Force = -U_potential_gradient
        #     print('Force')
        #     print(Force)
        #     k=0.01
        #     P = np.array([_eePoint[0], _eePoint[1]+k*Force, _eePoint[2]])
        #     # P = np.array([xCoord, 0, 0.5])
        #     print('P')
        #     print(P)
        #     return P
        #
        # st.addPointObjective(PotentialFunction(pointID, targetPosition, numFrames, potentialFunctionCalc), 7.5)


        # Create Solver and solve the Spacetime problem
        # print('st')
        # print(st)
        solver = SLSQPSolver(st,_callback=True)
        solution = solver()

        # Write solution to file
        writeTrajectoryFile(st, solution, "/home/rrl/Desktop/traj_legible.txt")

        rs = st.getStates(solution)
        print('robot joint states for legible trajectory')
        # print(rs)
        traj = []
        length = len(rs)
        for j in range(length):
            traj.append(rs[j])
        print(traj)

        return traj


def main():
    example = ExampleMoveItTrajectories()

    group = example.group
    joint_positions = group.get_current_joint_values()
    print('joint_positions start')
    print(joint_positions)

    pose_goal = group.get_current_pose().pose
    print('pose_goal')
    print(pose_goal)

    rospy.sleep(1)

    example.moveit()

    # if example.is_gripper_present and success:
    #     rospy.loginfo("Opening the gripper...")
    #     success &= example.reach_gripper_position(0)
    #     print('open gripper test')
    #     print (success)

    load_gazebo_models()

    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

    rospy.sleep(5)

    # pose_goal = group.get_current_pose().pose
    # # quaternion = np.array([0.0, 0.0, 0.0, 1.0])
    # quaternion = np.array([0.0, 1.0, 0.0, 0.0])
    # (roll, pitch, yaw) = euler_from_quaternion (quaternion)
    # print(math.degrees(roll))
    # print(math.degrees(pitch))
    # print(math.degrees(yaw))
    # # roll = roll - pi/4 goes to the side
    # # pitch = pitch + pi/4
    # pitch = pitch + pi/2
    # # yaw = yaw - pi/2 rotates gripper
    # print(math.degrees(roll))
    # print(math.degrees(pitch))
    # print(math.degrees(yaw))
    # quaternion = quaternion_from_euler(roll, pitch, yaw)
    # # pose_goal.position.x = 0.545
    # # pose_goal.position.y = 0.02
    # # pose_goal.position.z = 0.46
    # pose_goal.position.x = 0.745
    # pose_goal.position.y = 0.02
    # pose_goal.position.z = 0.7-0.78
    # pose_goal.orientation.x = quaternion[0]
    # pose_goal.orientation.y = quaternion[1]
    # pose_goal.orientation.z = quaternion[2]
    # pose_goal.orientation.w = quaternion[3]
    # group.set_pose_target(pose_goal)
    # ret = group.go(wait=True)
    # print(ret)
    # #
    # rospy.sleep(20)

    # rospy.Subscriber('/rotation_cup', Quaternion, example.rotation_cup_callback, queue_size=1)
    #
    # # while not subscribed yet sleep
    # while example.rotation_cup_data.x == 0.0 and example.rotation_cup_data.y == 0.0 and example.rotation_cup_data.z == 0.0:
    #     rospy.sleep(1)
    #
    # # check the rotation measured by pose estimation
    # print('example.rotation_cup_data.x')
    # print(example.rotation_cup_data.x)
    # pose_goal = group.get_current_pose().pose
    # q_rot = np.array([example.rotation_cup_data.x, example.rotation_cup_data.y, example.rotation_cup_data.z, example.rotation_cup_data.w])
    # # q_rot = np.array([0.0, 0.0, 0.0, 1.0])
    # (roll, pitch, yaw) = euler_from_quaternion (q_rot)
    # print(math.degrees(roll))
    # print(math.degrees(pitch))
    # print(math.degrees(yaw))
    # roll = roll - pi/2
    # pitch = pitch
    # yaw = yaw - pi/2
    # print(math.degrees(roll))
    # print(math.degrees(pitch))
    # print(math.degrees(yaw))
    # quaternion = quaternion_from_euler(roll, pitch, yaw)
    # # q_orig = np.array([pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w])
    # # (roll, pitch, yaw) = euler_from_quaternion (q_orig)
    # # print(math.degrees(roll))
    # # print(math.degrees(pitch))
    # # print(math.degrees(yaw))
    # # quaternion = quaternion_multiply(q_rot, q_orig)
    # # quaternion = q_rot
    # print('quaternion[0]')
    # print(quaternion[0])
    # pose_goal.orientation.x = quaternion[0]
    # pose_goal.orientation.y = quaternion[1]
    # pose_goal.orientation.z = quaternion[2]
    # pose_goal.orientation.w = quaternion[3]
    # print(' ')
    # print('quaternion')
    # print(quaternion)
    # print(' ')
    # group.set_pose_target(pose_goal)
    # ret = group.go(wait=True)
    # print(ret)
    #
    # rospy.sleep(10)
    #
    # pose_goal = group.get_current_pose().pose
    # # q_rot = np.array([example.rotation_cup_data.x, example.rotation_cup_data.y, example.rotation_cup_data.z, example.rotation_cup_data.w])
    # pose_goal.position.x = example.rotation_cup_data.x
    # pose_goal.position.y = example.rotation_cup_data.y
    # pose_goal.position.z = example.rotation_cup_data.z
    # print(' ')
    # print('quaternion')
    # print(quaternion)
    # print(' ')
    # group.set_pose_target(pose_goal)
    # ret = group.go(wait=True)
    # print(ret)

    # rospy.Subscriber('/ar_pose_marker',AlvarMarkers, example.artag_callback, queue_size=1)
    #
    # rospy.sleep(5)

    # while example.roll_gripper == 0.0 and example.pitch_gripper == 0.0 and example.roll_gripper == 0.0:
    #     rospy.sleep(5)
    #
    # print(' ')
    # print('pose_goal current')
    # print(pose_goal)
    # print(' ')
    #
    # print("roll_gripper:{}, pitch_gripper:{}, yaw_gripper:{}\n".format(example.roll_gripper,example.pitch_gripper,example.yaw_gripper))
    #
    # example.roll_gripper = example.roll_gripper - pi/2
    # example.pitch_gripper = example.pitch_gripper
    # example.yaw_gripper = example.yaw_gripper - pi/2

    # pose_goal = group.get_current_pose().pose
    #
    # q_rot = quaternion_from_euler(example.roll_gripper, example.pitch_gripper, example.yaw_gripper)
    # # q_orig = np.array([pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w])
    # # quaternion = quaternion_multiply(q_rot, q_orig)
    # quaternion = q_rot
    # # pose_goal.position.y = pose_goal.position.y + 0.1
    # pose_goal.orientation.x = quaternion[0]
    # pose_goal.orientation.y = quaternion[1]
    # pose_goal.orientation.z = quaternion[2]
    # pose_goal.orientation.w = quaternion[3]
    #
    # print(' ')
    # print('quaternion')
    # print(quaternion)
    # print(' ')

    # pose_goal.orientation.x = example.object_transform.transform.rotation.x
    # pose_goal.orientation.y = example.object_transform.transform.rotation.y
    # pose_goal.orientation.z = example.object_transform.transform.rotation.z
    # pose_goal.orientation.w = example.object_transform.transform.rotation.w

    # group.set_pose_target(pose_goal)
    # ret = group.go(wait=True)
    # print(ret)
    #
    # rospy.sleep(5)
    #
    # print('example.object_transform')
    # print(example.object_transform)


    pose_goal = group.get_current_pose().pose
    quaternion = np.array([0.0, 1.0, 0.0, 0.0])
    (roll, pitch, yaw) = euler_from_quaternion (quaternion)
    print(math.degrees(roll))
    print(math.degrees(pitch))
    print(math.degrees(yaw))
    roll = roll - 1.5
    pitch = pitch + pi/2
    print(math.degrees(roll))
    print(math.degrees(pitch))
    print(math.degrees(yaw))
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    pose_goal.position.x = 0.745
    pose_goal.position.y = 0.02
    pose_goal.position.z = 0.75-0.78
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    # pose_goal = group.get_current_pose().pose
    # quaternion = np.array([0.0, 0.0, 0.0, 1.0])
    # (roll, pitch, yaw) = euler_from_quaternion (quaternion)
    # print(math.degrees(roll))
    # print(math.degrees(pitch))
    # print(math.degrees(yaw))
    # # roll = roll - pi/2
    # # pitch = pitch
    # # yaw = yaw - pi/2
    # # print(math.degrees(roll))
    # # print(math.degrees(pitch))
    # # print(math.degrees(yaw))
    # quaternion = quaternion_from_euler(roll, pitch, yaw)
    # pose_goal.position.x = 0.745
    # pose_goal.position.y = 0.02
    # pose_goal.position.z = 0.86
    # pose_goal.orientation.x = quaternion[0]
    # pose_goal.orientation.y = quaternion[1]
    # pose_goal.orientation.z = quaternion[2]
    # pose_goal.orientation.w = quaternion[3]

    # <origin xyz="1.0 0 1.0" rpy="3.1416 2.3561 0"/>
    # (x=0.5, y=0, z=0.4)
    # pose_goal = group.get_current_pose().pose
    # todo: Modify!
    # pose_goal.position.x = example.object_transform.transform.translation.x
    # pose_goal.position.y = example.object_transform.transform.translation.y
    # pose_goal.position.z = example.object_transform.transform.translation.z
    # pose_goal.position.x = 0.445
    # pose_goal.position.y = 0.02
    # pose_goal.position.z = 0.445
    # pose_goal.position.x = example.apriltag_data.pose.position.x
    # pose_goal.position.y = example.apriltag_data.pose.position.y
    # pose_goal.position.z = example.apriltag_data.pose.position.z
    group.set_pose_target(pose_goal)
    # group.go(wait=True)
    plan2 = group.plan()
    print('pose_goal')
    print(pose_goal)

    print('plan2')
    print(plan2)

    # self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
    #                                                 moveit_msgs.msg.DisplayTrajectory,
    #                                                 queue_size=20)

    #
    # print('plan2 mod')
    # print(plan2[1])
    #
    # print('plan2 mod 2')
    # print(plan2[1].joint_trajectory)

    position_joints_start = plan2[1].joint_trajectory.points[0].positions
    # position_joints_start = plan2[1].joint_trajectory.points[0].positions

    print(plan2[1].joint_trajectory.points[-1].positions)

    position_joints_target = plan2[1].joint_trajectory.points[-1].positions

    print('calculate legible trajectory')
    traj = example.trajectory_legible_test(position_joints_start, position_joints_target)

    # plan each traj but don't execute; append trajs and change last velocity and acc value each

    # print(traj[1])
    # group.remember_joint_values('name1', [traj[1][0], traj[1][1], traj[1][2], traj[1][3], traj[1][4], traj[1][5], traj[1][6]] )
    # # group.remember_joint_values('name2', [traj[2][0], traj[2][1], traj[2][2], traj[2][3], traj[2][4], traj[2][5], traj[2][6]])
    # # group.remember_joint_values('name3', [traj[3][0], traj[3][1], traj[3][2], traj[3][3], traj[3][4], traj[3][5], traj[3][6]])
    # print(group.get_remembered_joint_values())

    # def moveit_commander.move_group.MoveGroupCommander.set_joint_value_target 	( 	  	self,
    # 		  	arg1,
    # 		  	arg2 = None,
    # 		  	arg3 = None
    # 	)
    #
    # Specify a target joint configuration for the group.
    # - if the type of arg1 is one of the following: dict, list, JointState message, then no other arguments should be provided.
    # The dict should specify pairs of joint variable names and their target values, the list should specify all the variable values
    # for the group. The JointState message specifies the positions of some single-dof joints.
    # - if the type of arg1 is string, then arg2 is expected to be defined and be either a real value or a list of real values. This is
    # interpreted as setting a particular joint to a particular value.
    # - if the type of arg1 is Pose or PoseStamped, both arg2 and arg3 could be defined. If arg2 or arg3 are defined, their types must
    # be either string or bool. The string type argument is interpreted as the end-effector the pose is specified for (default is to use
    # the default end-effector), and the bool is used to decide whether the pose specified is approximate (default is false). This situation
    # allows setting the joint target of the group by calling IK. This does not send a pose to the planner and the planner will do no IK.
    # Instead, one IK solution will be computed first, and that will be sent to the planner.

    # joint_positions1 = [traj[1][0], traj[1][1], traj[1][2], traj[1][3], traj[1][4], traj[1][5], traj[1][6]]
    # joint_positions2 = [traj[2][0], traj[2][1], traj[2][2], traj[2][3], traj[2][4], traj[2][5], traj[2][6]]
    # joint_positions = group.get_remembered_joint_values()
    joint_positions = group.get_current_joint_values()
    print('joint_positions start')
    print(joint_positions)
    joint_positions[0] = traj[len(traj)-1][0]
    joint_positions[1] = traj[len(traj)-1][1]
    joint_positions[2] = traj[len(traj)-1][2]
    joint_positions[3] = traj[len(traj)-1][3]
    joint_positions[4] = traj[len(traj)-1][4]
    joint_positions[5] = traj[len(traj)-1][5]
    joint_positions[6] = traj[len(traj)-1][6]
    group.set_joint_value_target(joint_positions)
    # group.set_joint_value_target(joint_positions1, joint_positions2)
    plan_traj = group.plan()
    print('plan_traj')
    print(plan_traj)
    print(' ')
    print('plan_traj points')
    print(plan_traj[1].joint_trajectory.points)
    print(' ')
    print('plan_traj len')
    print(len(plan_traj[1].joint_trajectory.points))
    print('traj length')
    print(len(traj))
    # index = int(len(plan_traj[1].joint_trajectory.points)/2)
    index = 0
    print('index')
    print(index)
    # print('plan_traj[1].joint_trajectory.points[index]')
    # print(plan_traj[1].joint_trajectory.points[index])

    # plan_traj_points = []

    # d = {}
    # for x in range(1, 10):
    #     d["string{0}".format(x)] = "Hello"

    # d = {}
    #
    # # while index < len(plan_traj[1].joint_trajectory.points):
    # for x in range(1, len(plan_traj[1].joint_trajectory.points)):
    #
    #     # plan_traj_points[index] = plan_traj[1].joint_trajectory.points[index]
    #     #
    #     # print('plan_traj_points[index]')
    #     # print(plan_traj_points[index])
    #
    #     # d["string{0}".format(x)] = "Hello"
    #
    #     d["point{0}".format(x)] = plan_traj[1].joint_trajectory.points[x]
    #
    #     print('x')
    #     print(x)
    #
    #     # index = index + 1
    #
    # print('d')
    # print(d)

    plan_traj_points = plan_traj[1].joint_trajectory.points

    print(int(len(plan_traj[1].joint_trajectory.points)/2))
    print(int(len(traj) - len(plan_traj[1].joint_trajectory.points)/2))

    minimum_add = int(len(plan_traj[1].joint_trajectory.points)/2)
    maximum_add = int(len(traj) - len(plan_traj[1].joint_trajectory.points)/2)

    max_sec = plan_traj_points[len(plan_traj_points)-1].time_from_start.secs
    # plan_traj_points[index].time_from_start.secs
    # print('max_sec')
    # print(max_sec)

    velocity_middle = plan_traj_points[int(len(plan_traj[1].joint_trajectory.points)/2)].velocities

    # add intermediate steps
    # while len(plan_traj[1].joint_trajectory.points) < len(traj):
    # while len(plan_traj[1].joint_trajectory.points) < len(traj):
    # while index < 10:
    while index < len(traj)-1:

        print('index')
        print(index)

        position_traj = traj[index]

        # print(int(len(plan_traj[1].joint_trajectory.points)/2))
        # print(int(len(traj) - len(plan_traj[1].joint_trajectory.points)/2))

        if index > minimum_add and index <= maximum_add:


            # # plan_traj_points[index].positions = position_traj
            # # plan_traj_points[index].time_from_start.secs = index
            #
            # # # trajectory_msgs/JointTrajectoryPoint.msg
            abc = trajectory_msgs.msg.JointTrajectoryPoint()
            print('abc')
            print(abc)
            abc.positions = position_traj
            abc.velocities = velocity_middle
            # # # position_traj = traj[index]
            # # # abc.positions = position_traj
            # abc.time_from_start.secs = index
            # #
            # # abc = plan_traj_points[index]
            # # abc.positions = position_traj
            # # abc.time_from_start.secs = index
            # # print('abc mod')
            # # print(abc)
            #
            plan_traj_points.insert(index, abc)

            print('plan_traj_points')
            print(plan_traj_points)

            # plan_traj_points.insert(index, plan_traj_points[index])
            # plan_traj[1].joint_trajectory.points = plan_traj_points

        else:
            plan_traj_points[index].positions = position_traj

        plan_traj_points[index].time_from_start.secs = round(index*(max_sec/(len(traj)-1)))
        index = index + 1
            # print('plan_traj')
            # print(plan_traj)


    # d = {}
    # ind = 0
    # for x in range(1, len(plan_traj[1].joint_trajectory.points)):
    #     d["point{0}".format(ind)] = plan_traj[1].joint_trajectory.points[x]
    #     ind = ind + 1
    #     d["point{0}".format(ind)] = 'fh'
    #     ind = ind + 1
    #     print('x')
    #     print(x)
    # print('d')
    # print(d)
    #
    #
    # # d['point5'] = 3
    # # plan_traj[1].joint_trajectory.points[x].position
    # key = d.get('point6')
    # print('key')
    # print(key)
    # key.positions = [1,2,3]
    # # key = key.replace('positions', 1)
    # # key = tuple(key)
    # # print(key.split())
    #
    # d['point6'] = key
    #
    # print('d mod')
    # print(d)

    #
    # # replace positions between start and end
    # index_position = 1

    # # while index_position < len(traj)-1:
    # while index_position < 2:
    #     position_planned = plan_traj[1].joint_trajectory.points[index_position].positions
    #     print('position_planned')
    #     print(position_planned)
    #     position_traj = traj[index_position]
    #     print('position_traj')
    #     print(position_traj)
    #     plan_traj_points = plan_traj[1].joint_trajectory.points
    #     # plan_traj_points[4].positions = position_traj
    #     # print('plan_traj_points[4]')
    #     # print(plan_traj_points[4])
    #     # plan_traj[1].joint_trajectory.points[4].positions = position_traj
    #     # print('type')
    #     # print(type(plan_traj[1].joint_trajectory.points[4].positions))
    #     # print(type(plan_traj[1].joint_trajectory.points[4]))
    #
    #     # y = list(plan_traj[1].joint_trajectory.points[4].positions)
    #     # y[1] = "kiwi"
    #     # x = tuple(y)
    #
    #     # # points works
    #     # plan_traj[1].joint_trajectory.points[4] = 'abc'
    #
    #     # plan_traj_points123 = plan_traj[1].joint_trajectory.points[4]
    #     # print('plan_traj_points123')
    #     # print(plan_traj_points123)
    #     # plan_traj_points123.positions = 'whop'
    #
    #     # a[:stop]
    #     plan_traj_points_first = plan_traj[1].joint_trajectory.points[:4]
    #     print(' ')
    #     print('plan_traj_points_first')
    #     print(plan_traj_points_first)
    #     print(' ')
    #     plan_traj_points_last = plan_traj[1].joint_trajectory.points[5:]
    #     print(' ')
    #     print('plan_traj_points_last')
    #     print(plan_traj_points_last)
    #     print(' ')
    #     plan_traj_points_middle = plan_traj[1].joint_trajectory.points[4]
    #     plan_traj_points_middle = list(plan_traj_points_middle)
    #     print(' ')
    #     print('plan_traj_points_middle')
    #     print(plan_traj_points_middle)
    #     print(' ')
    #
    #     # plan_traj_points_middle.positions = list(plan_traj_points_middle.positions)
    #     plan_traj_points_middle.positions = 'whop'
    #
    #     #
    #     # # # positions does not work
    #     # # plan_traj[1].joint_trajectory.points[4].positions = 'abc'
    #     #
    #     # print('plan_traj_points abc')
    #     # print(plan_traj_points)
    #     #
    #     # print('plan_traj abc')
    #     # print(plan_traj)
    #
    #     # plan_traj[1].joint_trajectory.points[index_position].positions = position_traj
    #
    #     # print('x')
    #     # print(x)
    #
    #     # print(plan_traj[1].joint_trajectory.points[4])
    #     # print(plan_traj[1].joint_trajectory.points.positions[4])
    #     # print(' ')
    #     # yy = plan_traj_points.points[:4]
    #     # y = list(yy)
    #     # zz = plan_traj_points.points[5:]
    #     # z = list(zz)
    #     # xyz = y + position_planned + z
    #     # plan_traj_points.points[4].positions = position_planned
    #     # x = tuple(xyz)
    #     # print('x')
    #     # print(x)
    #     # plan_traj = (plan_traj[1].joint_trajectory.points[:4].positions,) + position_planned + (plan_traj[1].joint_trajectory.points[5:].positions,)
    #     # plan_traj=plan_traj[1].joint_trajectory.points[:4].positions+position_traj+plan_traj[1].joint_trajectory.points[5:].positions
    #     # l=l[:index]+[‘new_value’]+l[index+1:]
    #     index_position = index_position + 1
    #     print('plan_traj[1].joint_trajectory.points[index_position].positions')
    #     print(plan_traj[1].joint_trajectory.points[index_position].positions)
    #     print(' ')
    #     print('plan_traj')
    #     print(plan_traj)
    #     print(' ')
    #     print('index_position')
    #     print(index_position)

    print('plan_traj final')
    print(plan_traj)






    # group = example.group
    # # length = 3
    # length = len(traj)
    # for j in range(length):
    #     if j == 0:
    #         joint_positions = group.get_current_joint_values()
    #     else:
    #         joint_positions = list(plan_traj[1].joint_trajectory.points[-1].positions)
    #         group.set_joint_value_target(joint_positions)
    #     print('joint_positions start')
    #     print(joint_positions)
    #     joint_positions[0] = traj[j][0]
    #     joint_positions[1] = traj[j][1]
    #     joint_positions[2] = traj[j][2]
    #     joint_positions[3] = traj[j][3]
    #     joint_positions[4] = traj[j][4]
    #     joint_positions[5] = traj[j][5]
    #     joint_positions[6] = traj[j][6]
    #     group.set_joint_value_target(joint_positions)
    #     plan_traj = group.plan()
    #     print('plan_traj')
    #     print(plan_traj)
    #     print('first traj joint_positions[0]')
    #     # print(plan_traj[1][0])
    #     # print(plan_traj[1].joint_trajectory.points[-1].positions)
    #     # print(list.positions))

    # https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html

    # waypoints = []
    # # wpose = move_group.get_current_pose().pose
    # # wpose.position.z -= scale * 0.1  # First move up (z)
    # # wpose.position.y += scale * 0.2  # and sideways (y)
    # # waypoints.append(copy.deepcopy(wpose))
    # # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    # # waypoints.append(copy.deepcopy(wpose))
    # # wpose.position.y -= scale * 0.1  # Third move sideways (y)
    # joint_positions = group.get_current_joint_values()
    # joint_positions[0] = traj[14][0]
    # joint_positions[1] = traj[14][1]
    # joint_positions[2] = traj[14][2]
    # joint_positions[3] = traj[14][3]
    # joint_positions[4] = traj[14][4]
    # joint_positions[5] = traj[14][5]
    # joint_positions[6] = traj[14][6]
    # waypoints.append(copy.deepcopy(joint_positions))
    # # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # # which is why we will specify 0.01 as the eef_step in Cartesian
    # # translation.  We will disable the jump threshold by setting it to 0.0,
    # # ignoring the check for infeasible jumps in joint space, which is sufficient
    # # for this tutorial.
    # (plan, fraction) = group.compute_cartesian_path(
    #     waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    # )  # jump_threshold
    # group.execute(plan, wait=True)

    # waypoints = []
    # wpose = move_group.get_current_pose().pose
    # wpose.position.z -= scale * 0.1  # First move up (z)
    # wpose.position.y += scale * 0.2  # and sideways (y)
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.y -= scale * 0.1  # Third move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))
    # # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # # which is why we will specify 0.01 as the eef_step in Cartesian
    # # translation.  We will disable the jump threshold by setting it to 0.0,
    # # ignoring the check for infeasible jumps in joint space, which is sufficient
    # # for this tutorial.
    # (plan, fraction) = move_group.compute_cartesian_path(
    #     waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    # )  # jump_threshold
    # move_group.execute(plan, wait=True)

    # print('all traj')
    # print(traj)
    # print('first traj length')
    # print(len(traj))
    # print('first traj')
    # print(traj[1])
    # print('first traj joint_positions[0]')
    # print(traj[1][0])
    # group = example.group
    # length = len(traj)
    # for j in range(length):
    #     joint_positions = group.get_current_joint_values()
    #     joint_positions[0] = traj[j][0]
    #     joint_positions[1] = traj[j][1]
    #     joint_positions[2] = traj[j][2]
    #     joint_positions[3] = traj[j][3]
    #     joint_positions[4] = traj[j][4]
    #     joint_positions[5] = traj[j][5]
    #     joint_positions[6] = traj[j][6]
    #     group.set_joint_value_target(joint_positions)
    #     group.go(wait=True)
    #     print('went to traj:',j)

    group = example.group
    joint_positions = group.get_current_joint_values()

    # print('joint_positions')
    # print(joint_positions)

    rospy.sleep(5)





if __name__ == '__main__':
    main()






# joint_positions start
# [0.08009761432595486, -0.9975894086301667, 1.187870052141494, 1.9419388095202024, -0.6699988326434614, 1.029902624627117, 0.4999423792629285]
# pose_goal
# position:
#   x: 0.578108490015891
#   y: -0.18399704872737382
#   z: 0.1120544511520577
# orientation:
#   x: -0.1404674361464546
#   y: 0.9897414582291519
#   z: -0.011344280988514067
#   w: 0.023495798214305374
# [ INFO] [1687990234.941904378, 17994.843000000]: ABORTED: CONTROL_FAILED
# 180.0
# -0.0
# 180.0
# 94.05633073037652
# 90.0
# 180.0
# pose_goal
# position:
#   x: 0.745
#   y: 0.02
#   z: -0.030000000000000027
# orientation:
#   x: -0.48199138953208925
#   y: 0.5173821608993934
#   z: 0.48199138953208936
#   w: 0.5173821608993933
# plan2
# (True, joint_trajectory:
#   header:
#     seq: 0
#     stamp:
#       secs: 0
#       nsecs:         0
#     frame_id: "world"
#   joint_names:
#     - right_s0
#     - right_s1
#     - right_e0
#     - right_e1
#     - right_w0
#     - right_w1
#     - right_w2
#   points:
#     -
#       positions: [0.07990078198719441, -0.997565131269428, 1.1880451871193536, 1.941975790068902, -0.6701176119352725, 1.029839190322141, 0.5000736451522281]
#       velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#       accelerations: [0.006793900421027673, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#       effort: []
#       time_from_start:
#         secs: 0
#         nsecs:         0
#     -
#       positions: [0.14048536044063986, -0.7806934887760819, 1.0725017262773398, 1.7120823545887087, -0.9241972869743185, 0.6075242415565588, 0.5488712665791373]
#       velocities: [0.014345828541123843, 0.05135305845252635, -0.02735954793448464, -0.05443649014844657, -0.060163552292362726, -0.1, 0.011554793778800317]
#       accelerations: [-1.6430613986527046e-18, 6.572245594610818e-18, 1.2322960489895283e-17, 0.0, 0.0, 0.0, 0.0]
#       effort: []
#       time_from_start:
#         secs: 4
#         nsecs: 223149488
#     -
#       positions: [0.20106993889408528, -0.5638218462827357, 0.9569582654353261, 1.4821889191085154, -1.1782769620133644, 0.18520929279097653, 0.5976688880060466]
#       velocities: [0.014345828541123845, 0.05135305845252636, -0.027359547934484633, -0.05443649014844659, -0.06016355229236275, -0.1, 0.011554793778800317]
#       accelerations: [2.4645920979790567e-18, -1.6430613986527046e-18, -1.067989909124258e-17, -9.858368391916227e-18, -9.858368391916227e-18, 0.0, -4.1076534966317614e-19]
#       effort: []
#       time_from_start:
#         secs: 8
#         nsecs: 446298975
#     -
#       positions: [0.26165451734753076, -0.34695020378938946, 0.8414148045933123, 1.2522954836283218, -1.4323566370524106, -0.23710565597460587, 0.6464665094329558]
#       velocities: [0.014345828541123843, 0.05135305845252636, -0.02735954793448464, -0.05443649014844659, -0.060163552292362754, -0.1, 0.011554793778800317]
#       accelerations: [-3.286122797305409e-18, -1.6430613986527046e-18, 9.036837692589875e-18, 8.215306993263523e-18, 6.572245594610818e-18, 0.0, 8.215306993263523e-19]
#       effort: []
#       time_from_start:
#         secs: 12
#         nsecs: 669448463
#     -
#       positions: [0.32223909580097615, -0.13007856129604334, 0.7258713437512987, 1.0224020481481284, -1.6864363120914565, -0.659420604740188, 0.695264130859865]
#       velocities: [0.014345828541123835, 0.05135305845252635, -0.02735954793448463, -0.05443649014844658, -0.06016355229236273, -0.1, 0.011554793778800313]
#       accelerations: [-4.1076534966317595e-19, 0.0, -4.929184195958112e-18, 0.0, 3.2861227973054076e-18, 0.0, -2.8753574476422317e-18]
#       effort: []
#       time_from_start:
#         secs: 16
#         nsecs: 892597951
#     -
#       positions: [0.3828236742544216, 0.086793081197303, 0.6103278829092849, 0.7925086126679348, -1.9405159871305027, -1.0817355535057707, 0.7440617522867743]
#       velocities: [0.014345828541123845, 0.05135305845252637, -0.02735954793448465, -0.054436490148446606, -0.060163552292362754, -0.1, 0.011554793778800317]
#       accelerations: [5.33994954562129e-18, 9.858368391916227e-18, -4.929184195958113e-18, -1.478755258787434e-17, -1.1501429790568932e-17, 0.0, 4.929184195958113e-18]
#       effort: []
#       time_from_start:
#         secs: 21
#         nsecs: 115747438
#     -
#       positions: [0.44340825270786705, 0.3036647236906492, 0.49478442206727113, 0.5626151771877413, -2.1945956621695486, -1.5040505022713526, 0.7928593737136835]
#       velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#       accelerations: [-0.0067939004210276835, -0.024319791947990638, 0.012956940318809963, 0.02578004416256796, 0.028492267426582757, 0.0473580204974027, -0.005472121606196871]
#       effort: []
#       time_from_start:
#         secs: 25
#         nsecs: 338896926
# multi_dof_joint_trajectory:
#   header:
#     seq: 0
#     stamp:
#       secs: 0
#       nsecs:         0
#     frame_id: ''
#   joint_names: []
#   points: [], 0.015656979, val: 1)
# (0.44340825270786705, 0.3036647236906492, 0.49478442206727113, 0.5626151771877413, -2.1945956621695486, -1.5040505022713526, 0.7928593737136835)
# calculate legible trajectory
# targetState
# [0.44340825270786705, 0.3036647236906492, 0.49478442206727113, 0.5626151771877413, -2.1945956621695486, -1.5040505022713526, 0.7928593737136835]
# Done! - stop criteria(0 = Optimization terminated successfully.)
# loops(428) evals(330) grads(98)
# minimum 279.15322464151365
# <Timer total(4.5 sec/1)>
# <Timer _slsqp(0.2 sec/429) Avg:0 Elap:4.5 range[0 0]>
# <Timer eval(0.7 sec/330) Avg:0 Elap:4.5 range[0 0]>
# <Timer evalG(3.3 sec/99) Avg:0 Elap:4.5 range[0 0.1]>
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~x from solver~~~~~~~~~~~~~~~~~~~~~~~~~
# [-1.99070176e-01 -7.44308653e-01  1.29883700e+00  1.85472495e+00
#  -7.67728878e-01  5.64671153e-01  2.36814958e-01 -3.55569506e-01
#  -5.55730951e-01  1.34494895e+00  1.76682683e+00 -8.65992896e-01
#   1.95536577e-01  6.95887897e-02 -4.09458694e-01 -4.22340746e-01
#   1.33587239e+00  1.68749434e+00 -9.55693904e-01 -8.96917256e-02
#  -1.37332677e-02 -3.99261559e-01 -3.22332985e-01  1.29341891e+00
#   1.61405325e+00 -1.03950250e+00 -3.22593594e-01 -4.47322025e-02
#  -3.52983857e-01 -2.39894332e-01  1.23339094e+00  1.54196006e+00
#  -1.12196376e+00 -5.25564389e-01 -4.57874978e-02 -2.87512686e-01
#  -1.65662131e-01  1.16515782e+00  1.46769765e+00 -1.20659177e+00
#  -7.10993400e-01 -2.93087617e-02 -2.12492140e-01 -9.44584560e-02
#   1.09390439e+00  1.38882713e+00 -1.29582469e+00 -8.84666776e-01
#  -1.08709900e-03 -1.33031726e-01 -2.37519039e-02  1.02215070e+00
#   1.30366790e+00 -1.39135499e+00 -1.04812870e+00  3.73543619e-02
#  -5.10373857e-02  4.70809494e-02  9.50520114e-01  1.21088271e+00
#  -1.49450350e+00 -1.19981331e+00  8.75731567e-02  3.41891896e-02
#   1.17007697e-01  8.77981668e-01  1.10899688e+00 -1.60675065e+00
#  -1.33529562e+00  1.53993603e-01  1.25449217e-01  1.83238483e-01
#   8.01745147e-01  9.95791962e-01 -1.73032087e+00 -1.44694629e+00
#   2.44252351e-01  2.26069924e-01  2.40917848e-01  7.16962214e-01
#   8.68096584e-01 -1.86838140e+00 -1.52319852e+00  3.69909027e-01
#   3.35921275e-01  2.83198503e-01  6.16786945e-01  7.23092059e-01
#  -2.02375457e+00 -1.54828915e+00  5.46712930e-01]
# Number of spins  0
# robot joint states for legible trajectory
# [array([ 0.07990078, -0.99756513,  1.18804519,  1.94197579, -0.67011761,
#         1.02983919,  0.50007365]), array([-0.19907018, -0.74430865,  1.298837  ,  1.85472495, -0.76772888,
#         0.56467115,  0.23681496]), array([-0.35556951, -0.55573095,  1.34494895,  1.76682683, -0.8659929 ,
#         0.19553658,  0.06958879]), array([-0.40945869, -0.42234075,  1.33587239,  1.68749434, -0.9556939 ,
#        -0.08969173, -0.01373327]), array([-0.39926156, -0.32233298,  1.29341891,  1.61405325, -1.0395025 ,
#        -0.32259359, -0.0447322 ]), array([-0.35298386, -0.23989433,  1.23339094,  1.54196006, -1.12196376,
#        -0.52556439, -0.0457875 ]), array([-0.28751269, -0.16566213,  1.16515782,  1.46769765, -1.20659177,
#        -0.7109934 , -0.02930876]), array([-2.12492140e-01, -9.44584560e-02,  1.09390439e+00,  1.38882713e+00,
#        -1.29582469e+00, -8.84666776e-01, -1.08709900e-03]), array([-0.13303173, -0.0237519 ,  1.0221507 ,  1.3036679 , -1.39135499,
#        -1.0481287 ,  0.03735436]), array([-0.05103739,  0.04708095,  0.95052011,  1.21088271, -1.4945035 ,
#        -1.19981331,  0.08757316]), array([ 0.03418919,  0.1170077 ,  0.87798167,  1.10899688, -1.60675065,
#        -1.33529562,  0.1539936 ]), array([ 0.12544922,  0.18323848,  0.80174515,  0.99579196, -1.73032087,
#        -1.44694629,  0.24425235]), array([ 0.22606992,  0.24091785,  0.71696221,  0.86809658, -1.8683814 ,
#        -1.52319852,  0.36990903]), array([ 0.33592127,  0.2831985 ,  0.61678694,  0.72309206, -2.02375457,
#        -1.54828915,  0.54671293]), array([ 0.44340825,  0.30366472,  0.49478442,  0.56261518, -2.19459566,
#        -1.5040505 ,  0.79285937])]
# joint_positions start
# [0.07990078165013692, -0.9975651303695399, 1.188045189707477, 1.9419757913102202, -0.6701176206959314, 1.0298391912886036, 0.5000736587097165]
# plan_traj
# (True, joint_trajectory:
#   header:
#     seq: 0
#     stamp:
#       secs: 0
#       nsecs:         0
#     frame_id: "world"
#   joint_names:
#     - right_s0
#     - right_s1
#     - right_e0
#     - right_e1
#     - right_w0
#     - right_w1
#     - right_w2
#   points:
#     -
#       positions: [0.07990078187092298, -0.9975651310583551, 1.1880451875916025, 1.9419757902197237, -0.6701176129389612, 1.0298391903449584, 0.5000736468821705]
#       velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#       accelerations: [0.006794954912527612, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#       effort: []
#       time_from_start:
#         secs: 0
#         nsecs:         0
#     -
#       positions: [0.14049887088329266, -0.7807080557446124, 1.0725007493863883, 1.71208664823969, -0.9242024850437107, 0.6075099293058328, 0.5488827998413388]
#       velocities: [0.014348541435009812, 0.05134786890687467, -0.02735885217162585, -0.054433628731857225, -0.060162743987850396, -0.1, 0.011557132659734553]
#       accelerations: [0.0, 6.572022868445663e-18, 0.0, 0.0, 0.0, 0.0, 6.161271439167809e-18]
#       effort: []
#       time_from_start:
#         secs: 4
#         nsecs: 223292610
#     -
#       positions: [0.20109695989566234, -0.5638509804308697, 0.9569563111811741, 1.4821975062596562, -1.1782873571484602, 0.1851806682667072, 0.5976919528005072]
#       velocities: [0.01434854143500982, 0.051347868906874684, -0.02735885217162585, -0.05443362873185725, -0.060162743987850396, -0.1, 0.011557132659734553]
#       accelerations: [3.2860114342228316e-18, 0.0, 0.0, -1.1501040019779912e-17, 0.0, 0.0, -6.161271439167809e-18]
#       effort: []
#       time_from_start:
#         secs: 8
#         nsecs: 446585221
#     -
#       positions: [0.2616950489080321, -0.3469939051171269, 0.8414118729759599, 1.2523083642796222, -1.4323722292532097, -0.2371485927724184, 0.6465011057596756]
#       velocities: [0.014348541435009812, 0.05134786890687467, -0.02735885217162585, -0.05443362873185725, -0.060162743987850396, -0.1, 0.01155713265973454]
#       accelerations: [-6.572022868445663e-18, -6.572022868445663e-18, 0.0, 1.1501040019779912e-17, 0.0, 0.0, 0.0]
#       effort: []
#       time_from_start:
#         secs: 12
#         nsecs: 669877831
#     -
#       positions: [0.3222931379204017, -0.13013682980338426, 0.7258674347707457, 1.0224192222995885, -1.6864571013579592, -0.659477853811544, 0.6953102587188439]
#       velocities: [0.014348541435009807, 0.051347868906874664, -0.02735885217162584, -0.05443362873185723, -0.060162743987850376, -0.1, 0.01155713265973455]
#       accelerations: [4.107514292778539e-18, 4.929017151334246e-18, 4.107514292778539e-18, -3.2860114342228312e-18, 9.858034302668493e-18, 0.0, 4.518265722056393e-18]
#       effort: []
#       time_from_start:
#         secs: 16
#         nsecs: 893170442
#     -
#       positions: [0.38289122693277144, 0.08672024551035862, 0.6103229965655315, 0.7925300803195545, -1.9405419734627087, -1.0818071148506698, 0.7441194116780123]
#       velocities: [0.014348541435009816, 0.05134786890687468, -0.02735885217162584, -0.05443362873185724, -0.06016274398785039, -0.1, 0.011557132659734553]
#       accelerations: [4.107514292778539e-19, 0.0, -4.107514292778539e-18, -1.6430057171114156e-18, -1.478705145400274e-17, 0.0, -3.2860114342228312e-18]
#       effort: []
#       time_from_start:
#         secs: 21
#         nsecs: 116463052
#     -
#       positions: [0.4434893159451411, 0.30357732082410127, 0.4947785583603173, 0.5626409383395208, -2.1946268455674582, -1.5041363758897952, 0.7929285646371806]
#       velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#       accelerations: [-0.0067949549125276175, -0.024316510194219158, 0.012956171734021183, 0.025777815440931245, 0.028490919070974258, 0.04735641558624365, -0.005473043772197385]
#       effort: []
#       time_from_start:
#         secs: 25
#         nsecs: 339755662
# multi_dof_joint_trajectory:
#   header:
#     seq: 0
#     stamp:
#       secs: 0
#       nsecs:         0
#     frame_id: ''
#   joint_names: []
#   points: [], 0.028517828000000002, val: 1)
# plan_traj points
# [positions: [0.07990078187092298, -0.9975651310583551, 1.1880451875916025, 1.9419757902197237, -0.6701176129389612, 1.0298391903449584, 0.5000736468821705]
# velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# accelerations: [0.006794954912527612, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# effort: []
# time_from_start:
#   secs: 0
#   nsecs:         0, positions: [0.14049887088329266, -0.7807080557446124, 1.0725007493863883, 1.71208664823969, -0.9242024850437107, 0.6075099293058328, 0.5488827998413388]
# velocities: [0.014348541435009812, 0.05134786890687467, -0.02735885217162585, -0.054433628731857225, -0.060162743987850396, -0.1, 0.011557132659734553]
# accelerations: [0.0, 6.572022868445663e-18, 0.0, 0.0, 0.0, 0.0, 6.161271439167809e-18]
# effort: []
# time_from_start:
#   secs: 4
#   nsecs: 223292610, positions: [0.20109695989566234, -0.5638509804308697, 0.9569563111811741, 1.4821975062596562, -1.1782873571484602, 0.1851806682667072, 0.5976919528005072]
# velocities: [0.01434854143500982, 0.051347868906874684, -0.02735885217162585, -0.05443362873185725, -0.060162743987850396, -0.1, 0.011557132659734553]
# accelerations: [3.2860114342228316e-18, 0.0, 0.0, -1.1501040019779912e-17, 0.0, 0.0, -6.161271439167809e-18]
# effort: []
# time_from_start:
#   secs: 8
#   nsecs: 446585221, positions: [0.2616950489080321, -0.3469939051171269, 0.8414118729759599, 1.2523083642796222, -1.4323722292532097, -0.2371485927724184, 0.6465011057596756]
# velocities: [0.014348541435009812, 0.05134786890687467, -0.02735885217162585, -0.05443362873185725, -0.060162743987850396, -0.1, 0.01155713265973454]
# accelerations: [-6.572022868445663e-18, -6.572022868445663e-18, 0.0, 1.1501040019779912e-17, 0.0, 0.0, 0.0]
# effort: []
# time_from_start:
#   secs: 12
#   nsecs: 669877831, positions: [0.3222931379204017, -0.13013682980338426, 0.7258674347707457, 1.0224192222995885, -1.6864571013579592, -0.659477853811544, 0.6953102587188439]
# velocities: [0.014348541435009807, 0.051347868906874664, -0.02735885217162584, -0.05443362873185723, -0.060162743987850376, -0.1, 0.01155713265973455]
# accelerations: [4.107514292778539e-18, 4.929017151334246e-18, 4.107514292778539e-18, -3.2860114342228312e-18, 9.858034302668493e-18, 0.0, 4.518265722056393e-18]
# effort: []
# time_from_start:
#   secs: 16
#   nsecs: 893170442, positions: [0.38289122693277144, 0.08672024551035862, 0.6103229965655315, 0.7925300803195545, -1.9405419734627087, -1.0818071148506698, 0.7441194116780123]
# velocities: [0.014348541435009816, 0.05134786890687468, -0.02735885217162584, -0.05443362873185724, -0.06016274398785039, -0.1, 0.011557132659734553]
# accelerations: [4.107514292778539e-19, 0.0, -4.107514292778539e-18, -1.6430057171114156e-18, -1.478705145400274e-17, 0.0, -3.2860114342228312e-18]
# effort: []
# time_from_start:
#   secs: 21
#   nsecs: 116463052, positions: [0.4434893159451411, 0.30357732082410127, 0.4947785583603173, 0.5626409383395208, -2.1946268455674582, -1.5041363758897952, 0.7929285646371806]
# velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# accelerations: [-0.0067949549125276175, -0.024316510194219158, 0.012956171734021183, 0.025777815440931245, 0.028490919070974258, 0.04735641558624365, -0.005473043772197385]
# effort: []
# time_from_start:
#   secs: 25
#   nsecs: 339755662]
# plan_traj len
# 7
# traj length
# 15
# index
# 3
# plan_traj[1].joint_trajectory.points[index]
# positions: [0.2616950489080321, -0.3469939051171269, 0.8414118729759599, 1.2523083642796222, -1.4323722292532097, -0.2371485927724184, 0.6465011057596756]
# velocities: [0.014348541435009812, 0.05134786890687467, -0.02735885217162585, -0.05443362873185725, -0.060162743987850396, -0.1, 0.01155713265973454]
# accelerations: [-6.572022868445663e-18, -6.572022868445663e-18, 0.0, 1.1501040019779912e-17, 0.0, 0.0, 0.0]
# effort: []
# time_from_start:
#   secs: 12
#   nsecs: 669877831
# Traceback (most recent call last):
#   File "/home/rrl/ros_ws_baxter_collision_detection/src/pick_and_place_baxter_moveit/src/example_move_it_trajectories.py", line 1151, in <module>
#     main()
#   File "/home/rrl/ros_ws_baxter_collision_detection/src/pick_and_place_baxter_moveit/src/example_move_it_trajectories.py", line 1016, in main
#     plan_traj_joint_trajectory = list(plan_traj[1].joint_trajectory)
# TypeError: 'JointTrajectory' object is not iterable
