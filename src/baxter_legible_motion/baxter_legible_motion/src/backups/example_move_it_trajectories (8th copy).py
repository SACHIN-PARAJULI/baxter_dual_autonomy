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
# from trajopt.utilities.fileOperations import writeTrajectoryFile
# from trajopt.spacetime.spacetime import Spacetime
# from trajopt.robot.builtinRobots import Particle3DRobot
# from trajopt.spacetime.builtinObjectives import StateVelocity, AlignAxis, LegibleS, LegibleG, PotentialFunction, PointVelocity
# from trajopt.spacetime.builtinConstraints import Nail, alignAxisGT, AboveFloor, pointDistance
# from trajopt.utilities.fileOperations import writeTrajectoryFile

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

from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity

from visualization_msgs.msg import Marker
# from visualization_msgs.msg import MarkerArray



def load_gazebo_models(table_pose=Pose(position=Point(x=0.7, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       # scissors_pose=Pose(position=Point(x=0.75, y=-0.1, z=0.78)),
                       # scissors_reference_frame="world",
                       # clock_pose=Pose(position=Point(x=0.65, y=0, z=0.78)),
                       # clock_reference_frame="world",
                       # bowl_pose=Pose(position=Point(x=0.75, y=0.2, z=0.78)),
                       # bowl_reference_frame="world",
                       # mug_pose=Pose(position=Point(x=0.77, y=0, z=0.78)),
                       # mug_reference_frame="world",
                       # tennis_ball_pose=Pose(position=Point(x=0.8, y=0.15, z=0.78)),
                       # tennis_ball_reference_frame="world"
                       mug0_pose=Pose(position=Point(x=0.77, y=0.22, z=0.78)),
                       mug0_reference_frame="world",
                       mug1_pose=Pose(position=Point(x=0.77, y=0.02, z=0.78)),
                       mug1_reference_frame="world",
                       mug2_pose=Pose(position=Point(x=0.77, y=-0.18, z=0.78)),
                       mug2_reference_frame="world",
                       mug3_pose=Pose(position=Point(x=0.77, y=-0.38, z=0.78)),
                       mug3_reference_frame="world",
                       mug4_pose=Pose(position=Point(x=0.77, y=-0.58, z=0.78)),
                       mug4_reference_frame="world"
                       ):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_gazebo')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')

    # # Load scissors URDF
    # scissors_xml = ''
    # with open (model_path + "037_scissors/scissors.sdf", "r") as scissors_file:
    #     scissors_xml=scissors_file.read().replace('\n', '')
    # # Spawn Table SDF
    # rospy.wait_for_service('/gazebo/spawn_sdf_model')
    #
    # # Load clock URDF
    # clock_xml = ''
    # with open (model_path + "076_timer/timer.sdf", "r") as clock_file:
    #     clock_xml=clock_file.read().replace('\n', '')
    # # Spawn Table SDF
    # rospy.wait_for_service('/gazebo/spawn_sdf_model')
    #
    # # Load bowl URDF
    # bowl_xml = ''
    # with open (model_path + "024_bowl/bowl.sdf", "r") as bowl_file:
    #     bowl_xml=bowl_file.read().replace('\n', '')
    # # Spawn Table SDF
    # rospy.wait_for_service('/gazebo/spawn_sdf_model')
    #
    # # Load mug URDF
    # mug_xml = ''
    # with open (model_path + "025_mug/mug.sdf", "r") as mug_file:
    # # with open (model_path + "mug_artag/mug_artag.sdf", "r") as mug_file:
    #     mug_xml=mug_file.read().replace('\n', '')
    # # Spawn Table SDF
    # rospy.wait_for_service('/gazebo/spawn_sdf_model')
    #
    # # Load tennis_ball URDF
    # tennis_ball_xml = ''
    # with open (model_path + "056_tennis_ball/tennis_ball.sdf", "r") as tennis_ball_file:
    #     tennis_ball_xml=tennis_ball_file.read().replace('\n', '')
    # # Spawn Table SDF
    # rospy.wait_for_service('/gazebo/spawn_sdf_model')

    # Load mug URDF
    mug0_xml = ''
    with open (model_path + "025_mug/mug.sdf", "r") as mug_file:
    # with open (model_path + "mug_artag/mug_artag.sdf", "r") as mug_file:
        mug0_xml=mug_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    # Load mug URDF
    mug1_xml = ''
    with open (model_path + "025_mug/mug.sdf", "r") as mug_file:
    # with open (model_path + "mug_artag/mug_artag.sdf", "r") as mug_file:
        mug1_xml=mug_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    # Load mug URDF
    mug2_xml = ''
    with open (model_path + "025_mug/mug.sdf", "r") as mug_file:
    # with open (model_path + "mug_artag/mug_artag.sdf", "r") as mug_file:
        mug2_xml=mug_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    # Load mug URDF
    mug3_xml = ''
    with open (model_path + "025_mug/mug.sdf", "r") as mug_file:
    # with open (model_path + "mug_artag/mug_artag.sdf", "r") as mug_file:
        mug3_xml=mug_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    # Load mug URDF
    mug4_xml = ''
    with open (model_path + "025_mug/mug.sdf", "r") as mug_file:
    # with open (model_path + "mug_artag/mug_artag.sdf", "r") as mug_file:
        mug4_xml=mug_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')


    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    # # Spawn scissors URDF
    # rospy.wait_for_service('/gazebo/spawn_urdf_model')
    # try:
    #     spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #     resp_sdf = spawn_sdf("scissors", scissors_xml, "/",
    #                          scissors_pose, scissors_reference_frame)
    # except rospy.ServiceException as e:
    #     rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    #
    # # Spawn clock URDF
    # rospy.wait_for_service('/gazebo/spawn_urdf_model')
    # try:
    #     spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #     resp_sdf = spawn_sdf("clock", clock_xml, "/",
    #                          clock_pose, clock_reference_frame)
    # except rospy.ServiceException as e:
    #     rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    #
    # # Spawn bowl URDF
    # rospy.wait_for_service('/gazebo/spawn_urdf_model')
    # try:
    #     spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #     resp_sdf = spawn_sdf("bowl", bowl_xml, "/",
    #                          bowl_pose, bowl_reference_frame)
    # except rospy.ServiceException as e:
    #     rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    #
    # # Spawn mug URDF
    # rospy.wait_for_service('/gazebo/spawn_urdf_model')
    # try:
    #     spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #     resp_sdf = spawn_sdf("mug", mug_xml, "/",
    #                          mug_pose, mug_reference_frame)
    # except rospy.ServiceException as e:
    #     rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    #
    # # Spawn tennis_ball URDF
    # rospy.wait_for_service('/gazebo/spawn_urdf_model')
    # try:
    #     spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #     resp_sdf = spawn_sdf("tennis_ball", tennis_ball_xml, "/",
    #                          tennis_ball_pose, tennis_ball_reference_frame)
    # except rospy.ServiceException as e:
    #     rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn mug0 URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("mug0", mug0_xml, "/",
                             mug0_pose, mug0_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn mug1 URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("mug1", mug1_xml, "/",
                             mug1_pose, mug1_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn mug2 URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("mug2", mug2_xml, "/",
                             mug2_pose, mug2_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn mug3 URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("mug3", mug3_xml, "/",
                             mug3_pose, mug3_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn mug4 URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("mug4", mug4_xml, "/",
                             mug4_pose, mug4_reference_frame)
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
        # resp_delete = delete_model("scissors")
        # resp_delete = delete_model("clock")
        # resp_delete = delete_model("bowl")
        # resp_delete = delete_model("mug")
        # resp_delete = delete_model("tennis_ball")
        resp_delete = delete_model("mug0")
        resp_delete = delete_model("mug1")
        resp_delete = delete_model("mug2")
        resp_delete = delete_model("mug3")
        resp_delete = delete_model("mug4")
    except rospy.ServiceException as e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


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

        self.gripper = baxter_interface.Gripper("right")
        self.gripper.calibrate()
        self.gripper.set_holding_force(100.0)

        # rviz markers
        self.marker = dict()
        self.pub_marker = rospy.Publisher('visualization_marker', Marker, queue_size=10)


    def marker_add(self, index, objectPosition):

        self.marker[index] = Marker()
        self.marker[index].header.frame_id = self.robot.get_planning_frame()
        self.marker[index].header.stamp = rospy.Time.now()
        self.marker[index].ns = "robot"
        self.marker[index].action = self.marker[index].ADD
        self.marker[index].type = self.marker[index].CUBE
        self.marker[index].id = index

        self.marker[index].scale.x = 0.1
        self.marker[index].scale.y = 0.1
        self.marker[index].scale.z = 0.1

        self.marker[index].color.a = 1.0
        self.marker[index].color.r = 0.0
        self.marker[index].color.g = 1.0
        self.marker[index].color.b = 0.0

        self.marker[index].pose.position.x = objectPosition[0]
        self.marker[index].pose.position.y = objectPosition[1]
        self.marker[index].pose.position.z = objectPosition[2]
        self.marker[index].pose.orientation.x = 0
        self.marker[index].pose.orientation.y = 0
        self.marker[index].pose.orientation.z = 0
        self.marker[index].pose.orientation.w = 1

        self.pub_marker.publish(self.marker[index])


    def marker_delete(self):
        for index in range(5):
            self.marker[index].action = self.marker[index].DELETE
            self.pub_marker.publish(self.marker[index])


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

        # move to home position
        joint_positions[0] = 0.08
        joint_positions[1] = -1.0
        joint_positions[2] = 1.19
        joint_positions[3] = 1.94
        joint_positions[4] = -0.67
        joint_positions[5] = 1.03
        joint_positions[6] = 0.50

        # joint_positions[0] = -0.78
        # joint_positions[1] = 0.0
        # joint_positions[2] = 3.0
        # joint_positions[3] = 1.57
        # joint_positions[4] = 0.0
        # joint_positions[5] = 0.0
        # joint_positions[6] = 1.57

        # joint_positions[0] = -0.78
        # joint_positions[1] = 0.0
        # joint_positions[2] = 1.57
        # joint_positions[3] = 1.57
        # joint_positions[4] = 0.0
        # joint_positions[5] = 0.0
        # joint_positions[6] = 1.57

        group.set_joint_value_target(joint_positions)
        # Plan and execute in one command
        group.go(wait=True)
        print('done move to home position')


    def trajectory_legible_test(self, startPosition, targetPosition, obstaclePositions):
        # Create Robot
        # points = (0.77,0.22,-0.03)
        # # beginning of whole thing
        # startpos=(0.58,-0.18,0.112)

        # parameters of the solution
        nsteps=15
        pointID = -1
        # noZ = True
        ptvel = 75
        ptvel2 = 1
        legw = 7.5

        # Create Robot
        # basic point robot problem
        robot = trajopt.robot.builtinRobots.Particle3DRobot(1)
        st = trajopt.spacetime.spacetime.Spacetime(robot, nsteps)

        # constraints
        c1 = trajopt.spacetime.builtinConstraints.Nail(robot.npoints - 1, startPosition, False)
        st.addConstraint(0, c1)
        c2 = trajopt.spacetime.builtinConstraints.Nail(robot.npoints - 1, targetPosition, False)
        st.addConstraint(nsteps - 1, c2)

        # radius = 0.1
        # st.addAllTimeConstraint(trajopt.spacetime.builtinConstraints.pointDistance(pointID,0.5,0.77,0.22,-0.03,False))
        # st.addAllTimeConstraint(trajopt.spacetime.builtinConstraints.pointDistance(pointID,0.5,0.77,0.02,-0.03,False))
        # st.addAllTimeConstraint(trajopt.spacetime.builtinConstraints.pointDistance(pointID,0.5,0.77,-0.18,-0.03,False))
        # st.addAllTimeConstraint(trajopt.spacetime.builtinConstraints.pointDistance(pointID,0.5,0.77,-0.38,-0.03,False))

        # objectives
        st.addPointObjective(trajopt.spacetime.builtinObjectives.PointVelocity(pointID), ptvel)

        def closePointToLine(_eePoint, _eeVel, _eeAlign, _pathRatio):
            yCoord = _eePoint[1]
            if (yCoord > 0.22):
                yCoord = 0.22
            elif (yCoord < -0.58):
                yCoord = -0.58
            zCoord = _eePoint[2]
            if (zCoord > 0.112):
                zCoord = 0.112
            elif (zCoord < -0.03):
                zCoord = -0.03
            P = np.array([0.77, yCoord, -zCoord])
            return P

        st.addPointObjective(trajopt.spacetime.builtinObjectives.LegibleG(pointID, targetPosition, nsteps, closePointToLine), legw)  # g vector term
        st.addPointObjective(trajopt.spacetime.builtinObjectives.LegibleS(pointID, targetPosition, nsteps), legw / 10)  # s vector term

        # Create Solver and solve the Spacetime problem
        solver = SLSQPSolver(st, _callback=True)
        solution = solver()


        # # Create Solver and solve the Spacetime problem
        # solver = SLSQPSolver(st,_callback=True)
        # solution = solver()


        # robot.default = position_joints_start
        #
        # # Create Spacetime Problem
        # numFrames = 15
        # last = numFrames-1
        # targetState = list(position_joints_target)
        # print('targetState')
        # print(targetState)
        #
        # st = Spacetime(robot, numFrames)
        # st.defaultState[numFrames - 1] = targetState
        # st.excludeKeys = [0]
        # st.excludeKeys.append(numFrames - 1)
        #
        # # Setup Objective (minimum joint velocity)
        # minJointVelocityTerm = StateVelocity()
        # minJointVelocityWeight = 100.0
        # st.addPointObjective(minJointVelocityTerm, minJointVelocityWeight) # add to objectives
        #
        # # Setup Objective (align end-effector axis upwards)
        # robotEndEffectorUpVector = (1, 0, 0)
        # # robotEndEffectorUpVector = (0, 0, 1)
        # # robotEndEffectorUpVector = (0, 1, 0)
        # # robotEndEffectorUpAxis = 0
        # robotEndEffectorUpAxis = 1
        # # robotEndEffectorUpAxis = 2
        # robotEndEffectorPoint = -1 # last point on robot
        #
        # alignAxisTerm = AlignAxis(robotEndEffectorPoint, robotEndEffectorUpAxis, robotEndEffectorUpVector)
        # minJointVelocityWeight = 25.0
        # st.addPointObjective(alignAxisTerm, minJointVelocityWeight) # add to objectives
        #
        # # Setup Constraints
        # # pointID = -1 # the point we are interested in
        # pointID = 0 # the point we are interested in
        # st.addConstraint(0, Nail(robotEndEffectorPoint, (0.57,-0.18,0.112), False))
        # st.addConstraint(numFrames - 1, Nail(robotEndEffectorPoint, targetPosition, False))
        # st.addConstraint(numFrames - 1, alignAxisGT(robotEndEffectorPoint, robotEndEffectorUpAxis, robotEndEffectorUpVector))
        # # st.addAllTimeConstraint(pointDistance(pointID,0.5,0.77,0.22,-0.03,False))
        # # st.addAllTimeConstraint(pointDistance(pointID,0.5,0.77,0.02,-0.03,False))
        # # st.addAllTimeConstraint(pointDistance(pointID,0.5,0.77,-0.18,-0.03,False))
        # # st.addAllTimeConstraint(pointDistance(pointID,0.5,0.77,-0.38,-0.03,False))
        #
        # # Calculate legible state of the art
        # legw = 7.5
        # # legw = 30
        # # take extrema of objects
        #    # mug0_pose=Pose(position=Point(x=0.77, y=0.22, z=0.78)),
        #    # mug1_pose=Pose(position=Point(x=0.77, y=0.02, z=0.78)),
        #    # mug2_pose=Pose(position=Point(x=0.77, y=-0.18, z=0.78)),
        #    # mug3_pose=Pose(position=Point(x=0.77, y=-0.38, z=0.78)),
        #    # mug4_pose=Pose(position=Point(x=0.77, y=-0.58, z=0.78)),
        # def closePointToLine(_eePoint, _eeVel, _eeAlign, _pathRatio):
        #     # print('_eePoint')
        #     # print(_eePoint)
        #     # print('_pathRatio')
        #     # print(_pathRatio)
        #     # xCoord = _eePoint[0]
        #     yCoord = _eePoint[1]
        #     # if (xCoord > 0.6):
        #     #     xCoord = 0.6
        #     # elif (xCoord < 0.45):
        #     #     xCoord = 0.45
        #     if (yCoord > 0.22):
        #         yCoord = 0.22
        #     elif (yCoord < -0.58):
        #         yCoord = -0.58
        #     # elif (yCoord < -0.1):
        #     #     yCoord = -0.1
        #     # if (xCoord > 5):
        #     #     xCoord = 5
        #     # elif (xCoord < -4):
        #     #     xCoord = -4
        #     # P = np.array([xCoord, 5, 0])
        #     # P = np.array([0.77, yCoord, 0.8])
        #     # P = np.array([0.745, yCoord, -0.03])
        #     P = np.array([0.77, yCoord, -0.03])
        #     # P = np.array([xCoord, yCoord, -0.03])
        #     # P = np.array([xCoord, 0, 0.5])
        #     # print('P')
        #     # print(P)
        #     return P
        # st.addPointObjective(LegibleG(pointID, targetPosition, numFrames, closePointToLine), legw)  # g vector term
        # st.addPointObjective(LegibleS(pointID, targetPosition, numFrames), legw / 10)  # s vector term
        #
        # # st.defaultState[last] = [targetPosition[0],targetPosition[1],targetPosition[2]]

        # # Create Solver and solve the Spacetime problem
        # solver = SLSQPSolver(st,_callback=True)
        # solution = solver()
        #
        # # Write solution to file
        # # writeTrajectoryFile(st, solution, "/home/melanie/Desktop/traj_legible.txt")



        # # ready2 = [-2.45697320422539, 2.957355675785974, 1.327502162859815, -1.243116391883747, -1.0383748231146699,
        # #           1.7725862364966851] # default state
        # robot = Baxter()
        # robot.default = position_joints_start
        #
        # # Create Spacetime Problem
        # numFrames = 15
        # targetPosition = (-0.300000, -0.500000, 0.060000)
        # targetState = position_joints_target
        #
        # st = Spacetime(robot, numFrames)
        # st.defaultState[numFrames - 1] = targetState
        # st.excludeKeys = [0]
        # st.excludeKeys.append(numFrames - 1)
        #
        # # Setup Objective (minimum joint velocity)
        # minJointVelocityTerm = StateVelocity()
        # minJointVelocityWeight = 100.0
        # st.addPointObjective(minJointVelocityTerm, minJointVelocityWeight) # add to objectives
        #
        # # Setup Objective (align end-effector axis upwards)
        # robotEndEffectorUpVector = (0, 0, 1)
        # robotEndEffectorUpAxis = 1
        # robotEndEffectorPoint = -1 # last point on robot
        #
        # alignAxisTerm = AlignAxis(robotEndEffectorPoint, robotEndEffectorUpAxis, robotEndEffectorUpVector)
        # minJointVelocityWeight = 25.0
        # st.addPointObjective(alignAxisTerm, minJointVelocityWeight) # add to objectives
        #
        # # Setup Constraints
        #
        # # Nail end position at target location
        # st.addConstraint(numFrames - 1, Nail(robotEndEffectorPoint, targetPosition, False))
        # st.addConstraint(numFrames - 1, alignAxisGT(robotEndEffectorPoint, robotEndEffectorUpAxis, robotEndEffectorUpVector))
        #
        # # Create Solver and solve the Spacetime problem
        # solver = SLSQPSolver(st,_callback=True)
        # solution = solver()

        # rs = st.getStates(solution)
        # print('robot joint states for legible trajectory')
        # # print(rs)
        # traj = []
        # length = len(rs)
        # for j in range(length):
        #     traj.append(rs[j])
        # print(traj)

        # return traj
        return solution

obstaclePosition_0 = np.array([0.77, -0.58, -0.03])
obstaclePosition_1 = np.asarray([0.77, 0.22, -0.03])
obstaclePosition_2 = np.asarray([0.77, 0.02, -0.03])
obstaclePosition_3 = np.asarray([0.77, -0.18, -0.03])
obstaclePosition_4 = np.asarray([0.77, -0.38, -0.03])
objects = [obstaclePosition_0, obstaclePosition_1, obstaclePosition_2, obstaclePosition_3, obstaclePosition_4]

def main(object_num):
    example = ExampleMoveItTrajectories()

    group = example.group
    joint_positions = group.get_current_joint_values()
    print('joint_positions start')
    print(joint_positions)

    pose_goal = group.get_current_pose().pose
    print('pose_goal')
    print(pose_goal)

    rospy.sleep(1)

    # uncomment
    example.moveit()

    # get target and obstacle positions
    obstaclePositions = []
    targetPosition = None
    for i in range(len(objects)):
        if i == object_num:
            targetPosition = objects[i]
        else:
            obstaclePositions.append(objects[i])

    objectPositions = copy.deepcopy(obstaclePositions)
    objectPositions.append(targetPosition)

    load_gazebo_models()

    for index in range(int(len(objectPositions))):
        example.marker_add(index, objectPositions[index])

    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

    rospy.on_shutdown(example.marker_delete)

    rospy.sleep(1)

    # waypoints = []
    # wpose = group.get_current_pose().pose
    # quaternion = [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]
    # wpose.position.x = targetPosition[0]
    # wpose.position.y = targetPosition[1]
    # wpose.position.z = targetPosition[2]
    # waypoints.append(copy.deepcopy(wpose))
    # print('waypoints')
    # print(waypoints)
    #
    # (plan2, fraction) = group.compute_cartesian_path(
    #                                    waypoints,   # waypoints to follow
    #                                    0.1,        # eef_step
    #                                    0.0)         # jump_threshold
    #
    #
    #

    # print('calculate state-of-the-art legible trajectory')
    # startPosition = (pose_goal.position.x, pose_goal.position.y, pose_goal.position.z)
    # solution = example.trajectory_legible_test(startPosition, targetPosition, obstaclePositions)
    #
    # x_positions = solution[0::3]
    # y_positions = solution[1::3]
    # z_positions = solution[2::3]
    #
    # waypoints = []
    # wpose = group.get_current_pose().pose
    # quaternion = [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]
    # # print(f'len(solution): {len(solution)/3}')
    # for i in range(int(len(solution)/3)):
    #     print(i)
    #     # pose = kdl_kin.forward(traj[i])
    #     # quat_updated = quaternion_from_matrix(pose)
    #     wpose.position.x = x_positions[i]
    #     wpose.position.y = y_positions[i]
    #     wpose.position.z = z_positions[i]
    #     wpose.orientation.x = quaternion[0]
    #     wpose.orientation.y = quaternion[1]
    #     wpose.orientation.z = quaternion[2]
    #     wpose.orientation.w = quaternion[3]
    #     waypoints.append(copy.deepcopy(wpose))
    #
    #
    # # position_joints_start = plan2.joint_trajectory.points[0].positions
    # # position_joints_target = plan2.joint_trajectory.points[-1].positions
    # # traj = example.trajectory_legible_test(position_joints_start, position_joints_target, targetPosition, obstaclePositions)
    # # robot = URDF.from_parameter_server()
    # # base_link = robot.get_root()
    # # end_link = 'right_gripper'
    # # kdl_kin = KDLKinematics(robot, base_link, end_link)
    # # waypoints = []
    # # wpose = group.get_current_pose().pose
    # # for i in range(len(traj)):
    # #     print(i)
    # #     pose = kdl_kin.forward(traj[i])
    # #     quat_updated = quaternion_from_matrix(pose)
    # #     wpose.position.x = pose.item((0, 3))
    # #     wpose.position.y = pose.item((1, 3))
    # #     wpose.position.z = pose.item((2, 3))
    # #     wpose.orientation.x = quat_updated[0]
    # #     wpose.orientation.y = quat_updated[1]
    # #     wpose.orientation.z = quat_updated[2]
    # #     wpose.orientation.w = quat_updated[3]
    # #     waypoints.append(copy.deepcopy(wpose))
    # # (plan2, fraction) = group.compute_cartesian_path(
    # #                                    waypoints,   # waypoints to follow
    # #                                    0.1,        # eef_step
    # #                                    0.0)         # jump_threshold
    # # ret = group.execute(plan2, wait=True)
    # # print(ret)
    # index = 0
    # file1_path = os.path.join(absolute_path,'Baxter_poses_legible.txt')
    # # print(file1_path)
    # file1 = open(file1_path, "w")
    # file1.close()
    # file1 = open(file1_path, "a")
    # while index < len(waypoints):
    #     # print current pose to file
    #     # writing newline character
    #     file1.write(str(waypoints[index]))
    #     file1.write("\n")
    #     index = index + 1
    # file1.close()
    # # rospy.sleep(5)
    # # example.moveit()


    # calculate potential field

    waypoints = []
    wpose = group.get_current_pose().pose
    quaternion = [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]
    wpose.position.x = targetPosition[0]
    wpose.position.y = targetPosition[1]
    wpose.position.z = targetPosition[2]
    waypoints.append(copy.deepcopy(wpose))
    print('waypoints')
    print(waypoints)

    (plan2, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.1,        # eef_step
                                       0.0)         # jump_threshold

    # calculate clutteredness of environment parameter
    # average distances between objects

   # mug0_pose=Pose(position=Point(x=0.77, y=0.22, z=0.78)),
   # mug1_pose=Pose(position=Point(x=0.77, y=0.02, z=0.78)),
   # mug2_pose=Pose(position=Point(x=0.77, y=-0.18, z=0.78)),
   # mug3_pose=Pose(position=Point(x=0.77, y=-0.38, z=0.78)),
   # mug4_pose=Pose(position=Point(x=0.77, y=-0.58, z=0.78)),

    # obstaclePosition_1 = np.asarray([0.77, 0.22, -0.03])
    # obstaclePosition_2 = np.asarray([0.77, 0.02, -0.03])
    # obstaclePosition_3 = np.asarray([0.77, -0.18, -0.03])
    # obstaclePosition_4 = np.asarray([0.77, -0.38, -0.03])

    # TODO: modify xi
    matrix = []
    for i in range(len(obstaclePositions)):
        row = []
        for j in range(len(obstaclePositions)):
             value = math.dist(obstaclePositions[i], obstaclePositions[j])
             row.append(value)
        matrix.append(row)
    matrix = np.tril(matrix)
    Sum = 0
    for i in range(len(obstaclePositions)):
        for j in range(len(obstaclePositions)):
            Sum += matrix[i][j]
    xi = Sum/6
    print('xi')
    print(xi)

    # sys.exit(0)

    robot = URDF.from_parameter_server()
    base_link = robot.get_root()
    end_link = 'right_gripper'
    kdl_kin = KDLKinematics(robot, base_link, end_link)
    print (kdl_kin)

    dof_robot = 7

    length_plan2 = len(plan2.joint_trajectory.points)

    plan2_original = copy.deepcopy(plan2)

    xyz_updated = np.zeros(shape=( (length_plan2-1)*3 ))
    xyz_prev = copy.deepcopy(xyz_updated)

    stop_rule_counter = 0

    # iter_points = copy.deepcopy(plan2[1:len(plan2) - 2])
    iter_points = []
    waypoints_orientation = []

    for i in range(len(plan2.joint_trajectory.points)):
        pose = kdl_kin.forward(np.asarray(plan2.joint_trajectory.points[i].positions))
        iter_points.append( [ np.asarray([ pose.item((0, 3)), pose.item((1, 3)), pose.item((2, 3)) ]), i] )


        quat_updated = quaternion_from_matrix(pose)
        waypoints_orientation.append( quat_updated )


    total_plan = copy.deepcopy(iter_points)

    # xy_array = np.array([1, 1, 0])
    # z_array = np.asarray([ 0, 0, 1 ])
    epsilon = 0.005
    k_rep = .060
    k_update = 0.2
    k_attr = 1.5
    ro_max = 0.03
    counter_total = 0
    counter = 0
    # iter_points
    print(f'iter_points start: {iter_points}')
    iter_points_index = 0

    # stop rule total plan:
    while True:

        total_plan_prev = copy.deepcopy(total_plan)
        length_iter_points_considered = len(iter_points)


        # stop rule moving points up
        while True:

            pose_updated = np.zeros(shape=( (length_plan2-1)*7 ))
            print(f'num_left: {len(iter_points)}')
            eeStartPosition = total_plan[0][0]

            # potential field updates
            for i in range(1,len(iter_points)-1):

                eePosition = iter_points[i][0]
                U_grad_rep_total = np.array([0.0, 0.0, 0.0])

                for j in obstaclePositions:

                    # get magnitude of force
                    # repulsive potential field w
                    ro_b = math.dist(eePosition, j)
                    ro_0 = math.dist(j, targetPosition)
                    if ro_b <= ro_0:
                        U_grad_rep = k_rep * (1/ro_0 - 1/ro_b ) * (1/(ro_b**2)) * ((eePosition-j)/ro_b)
                    else:
                        U_grad_rep = 0.0
                    print(f'U_grad_rep: {U_grad_rep}')

                    # additional force in z
                    j_z = np.array([ 0, 0, j[2]])
                    eePosition_z = np.array([ 0, 0, eePosition[2]])
                    U_grad_rep_z = - abs( k_rep * (1/ro_0 - 1/ro_b ) * (1/(ro_b**2)) * ((eePosition_z-j_z)/ro_b) )
                    print(f'U_grad_rep_z: {U_grad_rep_z}')

            		# get psi
                    psi = np.linalg.norm( np.cross( (j - eeStartPosition) , (targetPosition - eeStartPosition) ) ) / np.linalg.norm( (targetPosition - eeStartPosition) )
                    print(f'psi: {np.linalg.norm(psi)}')

            		# add them to the total
                    U_grad_rep_total += (U_grad_rep + U_grad_rep_z) * psi

                U_grad_attr = k_attr * (eePosition - targetPosition)

                print ('='* 30)
                print(U_grad_rep_total)
                print(U_grad_attr)
                print ('='* 30)
                # U_potential_gradient *= xi
                # print(f'\nxi: {xi}')

                force = -1 * (U_grad_rep_total + U_grad_attr)

                print(f'iter_points[i][0] before updated: {iter_points[i][0]}')
                iter_points[i][0] = iter_points[i][0] + k_update * force
                print(f'iter_points[i][0] updated: {iter_points[i][0]}')

                total_plan[iter_points[i][1]][0] = copy.deepcopy(iter_points[i][0])


            print(f'iter_points updated: {iter_points}')
            print(f'total_plan updated: {total_plan}')

            # move points 'up' to start position
            iter_points = copy.deepcopy(total_plan[iter_points_index:])
            print(f'iter_points copied: {iter_points}\n')
            print(f'total_plan_prev: {total_plan_prev}\n')

            if len(iter_points) > 1:
                for i in range(len(iter_points) - 1):
                    # if (np.linalg.norm(iter_points[i][0] - iter_points[i + 1][0])) == 0:
                    #     print('HELLO THIS IS ZERO PLEASE STOP')
                    #     continue
                    print(f'total_plan_prev index: {total_plan_prev[iter_points_index + i][0]}')
                    dir_vector = - (iter_points[i][0] - iter_points[i + 1][0]) / np.linalg.norm(iter_points[i][0] - iter_points[i + 1][0])
                    iter_points[i + 1][0] = dir_vector * ro_max + iter_points[i][0]
                    total_plan[iter_points[i][1]][0] = copy.deepcopy(iter_points[i][0])

            print(f'iter_points modified: {iter_points}')
            print(f'total_plan: {total_plan}')

            # stop rule and list changing
            to_remove = []
            for i in range(iter_points_index,len(total_plan)):
                print(f'np.linalg.norm(total_plan[i][0] - total_plan_prev[i][0]): {np.linalg.norm(total_plan[i][0] - total_plan_prev[i][0])}')
                if np.linalg.norm(total_plan[i][0] - total_plan_prev[i][0]) <= epsilon or np.linalg.norm(total_plan[i][0] - targetPosition) <= ro_max:
                    to_remove.append(i)
            print(f'to_remove: {to_remove}')

            for i in to_remove:
                for j in range(1,len(iter_points)-1):
                    if i == iter_points[j][1]:
                        del iter_points[j]
                        break

            print(f'iter_points: {iter_points}')

            if len(iter_points) == 2:
                print('\nbreak intermediate\n')
                break

            # get new previous points
            total_plan_prev = copy.deepcopy(total_plan)

            # delete later
            if counter >= 101:
                print('\ndoes not break\n')
                break
            counter = counter + 1

            print('\ncontinue\n')

        print(f'total_plan: {total_plan}')
        print(f'iter_points before: {iter_points}')

        # delete later
        if counter_total >= 10:
            print('\ndoes not break total\n')
            break

        # add points
        print(f'total_plan length before add: {len(total_plan)}')
        i = 1
        print(f'targetPosition: {targetPosition}')
        stop_rule_add_iter_points = np.linalg.norm(iter_points[-2][0] - targetPosition)
        print(f'stop_rule_add_iter_points: {stop_rule_add_iter_points}')
        iter_points = []
        length_total_plan = len(total_plan)
        max_dist = stop_rule_add_iter_points
        print(f'iter_points before 2: {iter_points}')
        dir_vector = - (total_plan[-2][0] - targetPosition) / np.linalg.norm(total_plan[-2][0] - targetPosition)
        while stop_rule_add_iter_points >= ro_max:
            print(f'stop_rule_add_iter_points: {stop_rule_add_iter_points}')
            if not iter_points:
                iter_point_new = total_plan[-2][0]
            else:
                iter_point_new = dir_vector * ro_max + iter_point_new
            iter_points.insert(i, [np.asarray([ iter_point_new[0], iter_point_new[1], iter_point_new[2] ]), length_total_plan - 3 + i])
            print(f'iter_points: {iter_points}')
            stop_rule_add_iter_points = np.linalg.norm(iter_point_new - targetPosition)
            i += 1
            # delete later
            if i == 20:
                print('could not end stop rule')
                break

        # add to total_plan
        total_plan = total_plan[:-1]
        length_total_plan = len(total_plan)
        length_iter_points = len(iter_points)
        total_plan[length_total_plan:length_iter_points-1] = iter_points[1:]
        total_plan[-1][1] = len(total_plan) - 1
        iter_points_index = iter_points[0][1]
        print(f'iter_points_index: {iter_points_index}')
        print(f'total_plan length after add: {len(total_plan)}')
        print(f'total_plan: {total_plan}')
        print(f'iter_points after: {iter_points}')

        if len(iter_points) == 2:
            print('\nbreak total\n')
            break

        # delete later
        if counter_total >= 10:
            print('\ndoes not break total\n')
            break
        counter_total = counter_total + 1

        print('\ncontinue total\n')



    # modify plan if moveit can't move there: use cartesian path
    # Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints.
    # Configurations are computed for every eef_step meters; The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resultingpath.
    # The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory.
    waypoints = []
    wpose = group.get_current_pose().pose

    print('pose_updated')
    print(pose_updated)

    # add orientation; modify later to point to target object?
    while len(waypoints_orientation) < len(total_plan):
        waypoints_orientation.append(waypoints_orientation[-1])

    for i in range(len(total_plan)):
    # for i in range(1):
        wpose.position.x = total_plan[i][0][0]
        wpose.position.y = total_plan[i][0][1]
        wpose.position.z = total_plan[i][0][2]
        wpose.orientation.x = waypoints_orientation[i][0]
        wpose.orientation.y = waypoints_orientation[i][1]
        wpose.orientation.z = waypoints_orientation[i][2]
        wpose.orientation.w = waypoints_orientation[i][3]

        # print(f'waypoints_orientation[i]: {waypoints_orientation[i]}')
        #
        # print(f'total_plan[i][0]: {total_plan[i][0]}')
        #
        # # calculate the orientation: robot end effector pointing to target
        # # dir_vector = - (total_plan[-2][0] - targetPosition) / np.linalg.norm(total_plan[-2][0] - targetPosition)
        # # https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
        # # dir_vector = (total_plan[i][0] - targetPosition) / np.linalg.norm(total_plan[i][0] - targetPosition)
        # # up_vector = np.array([0, 0, 1])
        # # cross_prod = np.cross(dir_vector, up_vector)
        # # q_w = math.sqrt(np.linalg.norm(dir_vector) * np.linalg.norm(up_vector) ) + np.dot(dir_vector, up_vector)
        # # # norm = np.linalg.norm(matrix)
        # # # matrix = matrix/norm  # normalized matrix
        # # quat_calc = np.array([cross_prod[0], cross_prod[1], cross_prod[2], q_w])
        # # norm = np.linalg.norm(quat_calc)
        # # quat_calc = quat_calc/norm
        #
        # dir_vector = - (total_plan[i][0] - targetPosition) / np.linalg.norm(total_plan[i][0] - targetPosition)
        # # up_vector = np.array([0, 0, 1])
        # up_vector = np.array([-1, 0, 0])
        # cross_prod = np.cross(dir_vector, up_vector)
        # q_w = math.sqrt(np.linalg.norm(dir_vector)**2 * np.linalg.norm(up_vector)**2 ) + np.dot(dir_vector, up_vector)
        # quat_calc = np.array([cross_prod[0], cross_prod[1], cross_prod[2], q_w])
        # norm = np.linalg.norm(quat_calc)
        # quat_calc = quat_calc/norm
        #
        # print(f'quat_calc: {quat_calc}')
        # angles = euler_from_quaternion([quat_calc[0], quat_calc[1], quat_calc[2], quat_calc[3]])
        # print(f'angles: {angles}')
        # angles_r = math.degrees(angles[0])
        # print(f'angles_r: {angles_r}')
        # angles_p = math.degrees(angles[1])
        # print(f'angles_p: {angles_p}')
        # angles_y = math.degrees(angles[2])
        # print(f'angles_y: {angles_y}')
        #
        # # Quaternion q;
        # # vector a = crossproduct(v1, v2);
        # # q.xyz = a;
        # # q.w = sqrt((v1.Length ^ 2) * (v2.Length ^ 2)) + dotproduct(v1, v2);
        #
        # print(f'targetPosition: {targetPosition}')
        # print(f'dir_vector: {dir_vector}')
        # print(f'up_vector: {up_vector}')
        # print(f'cross_prod: {cross_prod}')
        #
        # wpose.orientation.x = quat_calc[0]
        # wpose.orientation.y = quat_calc[1]
        # wpose.orientation.z = quat_calc[2]
        # wpose.orientation.w = quat_calc[3]

        waypoints.append(copy.deepcopy(wpose))
    wpose.position.x = targetPosition[0]
    wpose.position.y = targetPosition[1]
    wpose.position.z = targetPosition[2]
    waypoints.append(copy.deepcopy(wpose))


    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan2, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.1,        # eef_step
                                       0.0)         # jump_threshold

    # print('plan2')
    # print(plan2)
    print('fraction')
    print(fraction)



    robot_state = group.get_current_state()
    # print('robot_state')
    # print(robot_state)
    # plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=1.0, acceleration_scaling_factor=1.0)
    plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

    # uncomment
    # group.execute(plan2, wait=True)

    # print(plan2)
    # print(waypoints)

    index = 0
    file1_path = os.path.join(absolute_path,'Baxter_poses_potentialf.txt')
    file1 = open(file1_path, "w")
    file1.close()
    file1 = open(file1_path, "a")
    print('len(waypoints)')
    print(len(waypoints))
    while index < len(waypoints):
        # print current pose to file
        # writing newline character
        file1.write(str(waypoints[index]))
        file1.write("\n")
        index = index + 1
    file1.close()

    rospy.sleep(1)





if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        prog='ProgramName',
        description='What the program does',
        epilog='Text at the bottom of help')

    parser.add_argument('-on', '--object_number', default=0)

    args = parser.parse_args()

    main(int(args.object_number))
