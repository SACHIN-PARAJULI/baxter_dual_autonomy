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

    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = 0.4
    # pose_goal.position.y = 0.1
    # pose_goal.position.z = 0.4
    # group.set_pose_target(pose_goal)
    # # success = group.go(wait=True)
    # # plan = group.plan()
    # plan_success, plan, planning_time, error_code = group.plan()
    # group.execute(plan, wait=True)

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
    # plan2 = group.plan()
    plan_success, plan2, planning_time, error_code = group.plan()
    # group.execute(plan, wait=True)
    # group.go(wait=True)
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

    position_joints_start = plan2.joint_trajectory.points[0].positions
    # position_joints_start = plan2[1].joint_trajectory.points[0].positions
    # position_joints_start = plan2[1].joint_trajectory.points[0].positions
    print(position_joints_start)

    print(plan2.joint_trajectory.points[-1].positions)

    position_joints_target = plan2.joint_trajectory.points[-1].positions

    print('calculate legible trajectory')
    traj = example.trajectory_legible_test(position_joints_start, position_joints_target)

    # plan each traj but don't execute; append trajs and change last velocity and acc value each
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
    plan_success, plan_traj_orig, planning_time, error_code = group.plan()
    # plan_traj = group.plan()
    # print('plan_traj')
    # print(plan_traj)
    # print(' ')
    # print('plan_traj points')
    # print(plan_traj[1].joint_trajectory.points)
    # print(' ')
    # print('plan_traj len')
    # print(len(plan_traj[1].joint_trajectory.points))
    # print('traj length')
    # print(len(traj))

    plan_traj = copy.deepcopy(plan_traj_orig)

    index = 0
    # print('index')
    # print(index)

    plan_traj_points = plan_traj.joint_trajectory.points

    print(int(len(plan_traj.joint_trajectory.points)/2))
    print(int(len(traj) - len(plan_traj.joint_trajectory.points)/2))

    minimum_add = int(len(plan_traj.joint_trajectory.points)/2)
    maximum_add = int(len(traj) - len(plan_traj.joint_trajectory.points)/2)
    max_sec = plan_traj_points[len(plan_traj_points)-1].time_from_start.secs
    velocity_middle = plan_traj_points[int(len(plan_traj.joint_trajectory.points)/2)].velocities
    acceleration_middle = plan_traj_points[int(len(plan_traj.joint_trajectory.points)/2)].accelerations

    # add intermediate steps
    while index < len(traj):
    # while index < 5:

        # print('index')
        # print(index)

        position_traj = traj[index]

        if index > minimum_add and index <= maximum_add:

            abc = trajectory_msgs.msg.JointTrajectoryPoint()
            # print('abc')
            # print(abc)
            abc.positions = position_traj
            abc.velocities = velocity_middle
            abc.accelerations = acceleration_middle

            plan_traj_points.insert(index, abc)

            # print('plan_traj_points')
            # print(plan_traj_points)

        else:
            plan_traj_points[index].positions = position_traj

        plan_traj_points[index].time_from_start.secs = round(index*(max_sec/(len(traj)-1)))
        # plan_traj_points[index].time_from_start.secs = round(index*4)
        index = index + 1
            # print('plan_traj')
            # print(plan_traj)

    print('plan_traj final')
    print(plan_traj)

    # (success, trajectory, planning_time, error_code) = move_group.plan(goal_robot_state.joint_state)
    robot_state = group.get_current_state()
    print('robot_state')
    print(robot_state)
    # plan_traj = group.retime_trajectory(robot_state, plan_traj, velocity_scaling_factor=1.0, acceleration_scaling_factor=1.0)
    # plan_traj = group.retime_trajectory(robot_state, plan_traj, velocity_scaling_factor=0.1, acceleration_scaling_factor=0.1)
    #
    # print('plan_traj retimed')
    # print(plan_traj)

    # moveit_robot_state = moveit_msgs.msg.RobotState()
    # print( group.retime_trajectory(moveit_robot_state, plan_traj, velocity_scaling_factor=1.0, acceleration_scaling_factor=1.0, algorithm="iterative_time_parameterization") )

    print('plan_traj_orig')
    print(plan_traj_orig)

    # plan2 = plan_traj

    # group.execute(plan2, wait=True)



    # plan2_part = moveit_msgs.msg.RobotTrajectory()
    # efg = plan_traj[0]
    # plan2_part.joint_trajectory.points = copy.deepcopy(plan_traj.joint_trajectory.points)

    index = 0
    file1 = open("/home/rrl/Desktop/Baxter_poses.txt", "w")
    file1.close()
    file1 = open("/home/rrl/Desktop/Baxter_poses.txt", "a")
    while index < len(traj)-1:

        plan2_part = copy.deepcopy(plan_traj)

        indexes = [index, index+1]
        plan2_part.joint_trajectory.points = [plan2_part.joint_trajectory.points[x] for x in indexes]
        # print( [plan2_part.joint_trajectory.points[x] for x in indexes] )
        print(' ')
        print('plan2_part')
        print(plan2_part)
        print(' ')

        group.execute(plan2_part, wait=True)

        pose_current = group.get_current_pose().pose
        # print current pose to file

        # writing newline character
        file1.write("\n")
        file1.write(str(pose_current))

        index = index + 1

    file1.close()

    # group = example.group
    joint_positions = group.get_current_joint_values()

    print('joint_positions')
    print(joint_positions)

    rospy.sleep(5)





if __name__ == '__main__':
    main()
