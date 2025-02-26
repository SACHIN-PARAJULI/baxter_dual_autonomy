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
from trajopt.spacetime.spacetime import Spacetime
from trajopt.robot.builtinRobots import Baxter
from trajopt.spacetime.builtinObjectives import StateVelocity, AlignAxis, LegibleS, LegibleG, PotentialFunction
from trajopt.spacetime.builtinConstraints import Nail, alignAxisGT, AboveFloor, pointDistance
from trajopt.utilities.fileOperations import writeTrajectoryFile

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


    def trajectory_legible_test(self, position_joints_start, position_joints_target):
        # Create Robot
        robot = Baxter()
        robot.default = position_joints_start

        # Create Spacetime Problem
        numFrames = 15
        last = numFrames-1
        # numFrames = 60
            # pose_goal.position.x = 0.745
            # # pose_goal.position.y = 0.02
            # pose_goal.position.y = -0.58
            # pose_goal.position.z = 0.75-0.78
        # targetPosition = (0.77, -0.58, 0.8)
        targetPosition = (0.745, -0.58, -0.03)
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
        # robotEndEffectorUpVector = (1, 0, 0)
        robotEndEffectorUpVector = (0, 0, 1)
        # robotEndEffectorUpAxis = 1
        robotEndEffectorUpAxis = 2
        robotEndEffectorPoint = -1 # last point on robot

        alignAxisTerm = AlignAxis(robotEndEffectorPoint, robotEndEffectorUpAxis, robotEndEffectorUpVector)
        minJointVelocityWeight = 25.0
        st.addPointObjective(alignAxisTerm, minJointVelocityWeight) # add to objectives

        # Setup Constraints
        pointID = -1 # the point we are interested in
        # pointID = 0 # the point we are interested in
        st.addConstraint(numFrames - 1, Nail(robotEndEffectorPoint, targetPosition, False))
        # st.addConstraint(round(numFrames/2), alignAxisGT(robotEndEffectorPoint, robotEndEffectorUpAxis, robotEndEffectorUpVector))
        # i = 0
        # while i<=numFrames-1:
        #     st.addConstraint(i, alignAxisGT(robotEndEffectorPoint, robotEndEffectorUpAxis, robotEndEffectorUpVector))
        #     i = i + 1
        st.addConstraint(numFrames - 1, alignAxisGT(robotEndEffectorPoint, robotEndEffectorUpAxis, robotEndEffectorUpVector))

        # Calculate legible state of the art
        legw = 7.5
        # legw = 30
        # take extrema of objects
           # mug0_pose=Pose(position=Point(x=0.77, y=0.22, z=0.78)),
           # mug1_pose=Pose(position=Point(x=0.77, y=0.02, z=0.78)),
           # mug2_pose=Pose(position=Point(x=0.77, y=-0.18, z=0.78)),
           # mug3_pose=Pose(position=Point(x=0.77, y=-0.38, z=0.78)),
           # mug4_pose=Pose(position=Point(x=0.77, y=-0.58, z=0.78)),
        def closePointToLine(_eePoint, _eeVel, _eeAlign, _pathRatio):
            # print('_eePoint')
            # print(_eePoint)
            # print('_pathRatio')
            # print(_pathRatio)
            # xCoord = _eePoint[0]
            yCoord = _eePoint[1]
            # if (xCoord > 0.6):
            #     xCoord = 0.6
            # elif (xCoord < 0.45):
            #     xCoord = 0.45
            if (yCoord > 0.22):
                yCoord = 0.22
            elif (yCoord < -0.58):
                yCoord = -0.58
            # elif (yCoord < -0.1):
            #     yCoord = -0.1
            # if (xCoord > 5):
            #     xCoord = 5
            # elif (xCoord < -4):
            #     xCoord = -4
            # P = np.array([xCoord, 5, 0])
            # P = np.array([0.77, yCoord, 0.8])
            P = np.array([0.745, yCoord, -0.03])
            # P = np.array([xCoord, 0, 0.5])
            # print('P')
            # print(P)
            return P
        st.addPointObjective(LegibleG(pointID, targetPosition, numFrames, closePointToLine), legw)  # g vector term
        st.addPointObjective(LegibleS(pointID, targetPosition, numFrames), legw / 10)  # s vector term

        # st.defaultState[last] = [targetPosition[0],targetPosition[1],targetPosition[2]]

        # Create Solver and solve the Spacetime problem
        solver = SLSQPSolver(st,_callback=True)
        solution = solver()

        # Write solution to file
        # writeTrajectoryFile(st, solution, "/home/melanie/Desktop/traj_legible.txt")

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

    load_gazebo_models()

    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

    rospy.sleep(5)



    pose_goal = group.get_current_pose().pose
    quaternion = [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
    # quaternion = np.array([0.0, 0.0, 0.0, 1.0])
    # quaternion = np.array([0.0, 1.0, 0.0, 0.0])
    # (roll, pitch, yaw) = euler_from_quaternion (quaternion)
    # print(math.degrees(roll))
    # print(math.degrees(pitch))
    # print(math.degrees(yaw))
    # roll = roll - 1.5
    # pitch = pitch + pi/2
    # print(math.degrees(roll))
    # print(math.degrees(pitch))
    # print(math.degrees(yaw))
    # quaternion = quaternion_from_euler(roll, pitch, yaw)
    pose_goal.position.x = 0.77
    # pose_goal.position.y = 0.02
    pose_goal.position.y = -0.58
    pose_goal.position.z = 0.75-0.78
    # pose_goal.orientation.x = quaternion[0]
    # pose_goal.orientation.y = quaternion[1]
    # pose_goal.orientation.z = quaternion[2]
    # pose_goal.orientation.w = quaternion[3]
    group.set_pose_target(pose_goal)
    # ret = group.go(wait=True)
    plan_success, plan2, planning_time, error_code = group.plan()

    position_joints_start = plan2.joint_trajectory.points[0].positions
    # position_joints_start = plan2[1].joint_trajectory.points[0].positions
    # position_joints_start = plan2[1].joint_trajectory.points[0].positions
    print(position_joints_start)

    print(plan2.joint_trajectory.points[-1].positions)

    position_joints_target = plan2.joint_trajectory.points[-1].positions

    # print('calculate legible trajectory')
    # traj = example.trajectory_legible_test(position_joints_start, position_joints_target)
    #
    # robot = URDF.from_parameter_server()
    # base_link = robot.get_root()
    # end_link = 'right_gripper'
    # kdl_kin = KDLKinematics(robot, base_link, end_link)
    # waypoints = []
    # wpose = group.get_current_pose().pose
    # for i in range(0,(len(traj)-1)):
    #     print(i)
    #     pose = kdl_kin.forward(traj[i])
    #     quat_updated = quaternion_from_matrix(pose)
    #     # pose_index = np.asarray([ pose.item((0, 3)), pose.item((1, 3)), pose.item((2, 3)), quat_updated[0], quat_updated[1], quat_updated[2], quat_updated[3] ])
    #     wpose.position.x = pose.item((0, 3))
    #     wpose.position.y = pose.item((1, 3))
    #     wpose.position.z = pose.item((2, 3))
    #     wpose.orientation.x = quat_updated[0]
    #     wpose.orientation.y = quat_updated[1]
    #     wpose.orientation.z = quat_updated[2]
    #     wpose.orientation.w = quat_updated[3]
    #     waypoints.append(copy.deepcopy(wpose))
    # # append target position
    # wpose.position.x = 0.745
    # wpose.position.y = -0.58
    # wpose.position.z = 0.75-0.78
    # # wpose.orientation.x = quaternion[0]
    # # wpose.orientation.y = quaternion[1]
    # # wpose.orientation.z = quaternion[2]
    # # wpose.orientation.w = quaternion[3]
    # waypoints.append(copy.deepcopy(wpose))
    # print('waypoints')
    # print(waypoints)
    # print(len(waypoints))
    # (plan2, fraction) = group.compute_cartesian_path(
    #                                    waypoints,   # waypoints to follow
    #                                    0.01,        # eef_step
    #                                    0.0)         # jump_threshold
    # ret = group.execute(plan2, wait=True)
    # print(ret)
    #
    # index = 0
    # # sys.path.append(os.path.join(absolute_path,'hrl-kdl','pykdl_utils','src'))
    # # file1 = open("/home/melanie/Desktop/Baxter_poses_legible.txt", "w")
    # file1_path = os.path.join(absolute_path,'Baxter_poses_legible.txt')
    # # print(file1_path)
    # file1 = open(file1_path, "w")
    # file1.close()
    # file1 = open(file1_path, "a")
    # # file1 = open("/home/melanie/Desktop/Baxter_poses_legible.txt", "a")
    # while index < len(waypoints)-1:
    #     # print current pose to file
    #     # writing newline character
    #     file1.write("\n")
    #     file1.write(str(waypoints[index]))
    #     index = index + 1
    # file1.close()
    #
    # rospy.sleep(5)
    # example.moveit()




    #
    #
    # waypoints = []
    # wpose = group.get_current_pose().pose
    # # for i in range(0,(length_plan2-1)*7,7):
    # #     wpose.position.x = pose_updated[i]
    # #     wpose.position.y = pose_updated[i+1]
    # #     wpose.position.z = pose_updated[i+2]
    # #     wpose.orientation.x = pose_updated[i+3]
    # #     wpose.orientation.y = pose_updated[i+4]
    # #     wpose.orientation.z = pose_updated[i+5]
    # #     wpose.orientation.w = pose_updated[i+6]
    # #     waypoints.append(copy.deepcopy(wpose))
    # # append target position
    # wpose.position.x = 0.745
    # wpose.position.y = 0.02
    # wpose.position.z = 0.75-0.78
    # wpose.orientation.x = quaternion[0]
    # wpose.orientation.y = quaternion[1]
    # wpose.orientation.z = quaternion[2]
    # wpose.orientation.w = quaternion[3]
    # waypoints.append(copy.deepcopy(wpose))
    # print('waypoints')
    # print(waypoints)

    waypoints = []
    wpose = group.get_current_pose().pose
    # for i in range(0,(length_plan2-1)*7,7):
    #     wpose.position.x = pose_updated[i]
    #     wpose.position.y = pose_updated[i+1]
    #     wpose.position.z = pose_updated[i+2]
    #     wpose.orientation.x = pose_updated[i+3]
    #     wpose.orientation.y = pose_updated[i+4]
    #     wpose.orientation.z = pose_updated[i+5]
    #     wpose.orientation.w = pose_updated[i+6]
    #     waypoints.append(copy.deepcopy(wpose))
    # append target position
    wpose.position.x = 0.77
    wpose.position.y = -0.58
    wpose.position.z = 0.75-0.78
    wpose.orientation.x = quaternion[0]
    wpose.orientation.y = quaternion[1]
    wpose.orientation.z = quaternion[2]
    wpose.orientation.w = quaternion[3]
    waypoints.append(copy.deepcopy(wpose))
    print('waypoints')
    print(waypoints)

    (plan2, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    print('plan2')
    print(plan2)
    print('fraction')
    print(fraction)

    #
    #
    # # group.set_pose_target(pose_goal)
    # # # group.go(wait=True)
    # # # plan2 = group.plan()
    # # plan_success, plan2, planning_time, error_code = group.plan()
    # # # group.execute(plan, wait=True)
    # # # group.go(wait=True)
    # # print('pose_goal')
    # # print(pose_goal)
    # #
    # # print('plan others')
    # # print(plan_success)
    # # print(planning_time)
    # # print(error_code)
    #
    print('plan2')
    print(plan2)
    #
    # print('length plan2')
    # length_plan2 = len(plan2.joint_trajectory.points)
    # print(length_plan2)
    #
    # plan2_test = moveit_msgs.msg.RobotTrajectory()
    # print('plan2_test')
    # print(plan2_test)
    #
    # # def plan(self, joints=None):
    # #     """Return a tuple of the motion planning results such as
    # #     (success flag : boolean, trajectory message : RobotTrajectory,
    # #      planning time : float, error code : MoveitErrorCodes)"""
    # # thistuple = ("apple", "banana", "cherry")
    #
    # # plan_success, plan3, planning_time, error_code = group.plan( (plan_success, plan2, planning_time, error_code) )
    #
    # calculate potential field

    # calculate clutteredness of environment parameter
    # average distances between objects
    # create bottom triangle matrix with distances between objects
    # position of objects
    # scissors_pose=Pose(position=Point(x=0.75, y=-0.1, z=0.78)),
    # clock_pose=Pose(position=Point(x=0.65, y=0, z=0.78)),
    # bowl_pose=Pose(position=Point(x=0.75, y=0.2, z=0.78)),
    # tennis_ball_pose=Pose(position=Point(x=0.8, y=0.15, z=0.78)),
    # mug_pose=Pose(position=Point(x=0.77, y=0, z=0.78)),

           # mug0_pose=Pose(position=Point(x=0.77, y=0.22, z=0.78)),
           # mug1_pose=Pose(position=Point(x=0.77, y=0.02, z=0.78)),
           # mug2_pose=Pose(position=Point(x=0.77, y=-0.18, z=0.78)),
           # mug3_pose=Pose(position=Point(x=0.77, y=-0.38, z=0.78)),
           # mug4_pose=Pose(position=Point(x=0.77, y=-0.58, z=0.78)),
    obstaclePosition_1 = np.asarray([0.77, 0.22, 0.78])
    obstaclePosition_2 = np.asarray([0.77, 0.02, 0.78])
    obstaclePosition_3 = np.asarray([0.77, -0.18, 0.78])
    obstaclePosition_4 = np.asarray([0.77, -0.38, 0.78])
    x12 = math.dist(obstaclePosition_1, obstaclePosition_2)
    x13 = math.dist(obstaclePosition_1, obstaclePosition_3)
    x23 = math.dist(obstaclePosition_2, obstaclePosition_3)
    x14 = math.dist(obstaclePosition_1, obstaclePosition_4)
    x24 = math.dist(obstaclePosition_2, obstaclePosition_4)
    x34 = math.dist(obstaclePosition_3, obstaclePosition_4)
    arr_list = np.array([x12,x13,x23,x14,x24,x34])
    xi = sum(arr_list)/6
    # arr_list = np.array([obstaclePosition_1,obstaclePosition_2,obstaclePosition_3,obstaclePosition_4])
    # xi = sum(arr_list)/4
    print('xi')
    print(xi)



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

    # iter_points = [ [np.array([]), i]  for i in range( len(iter_points) ) ]

    iter_points_prev = copy.deepcopy(iter_points)
    total_plan = copy.deepcopy(iter_points)

    # print(iter_points)
    # print(total_plan)
    #
    # sys.exit(0)

    current_pose = group.get_current_pose().pose

    # np.set_printoptions(threshold=sys.maxsize)
    obstaclePositions = [ np.array([0.77, 0.22, 0.78]), np.array([0.77, 0.02, 0.78]), np.array([0.77, -0.18, 0.78]), np.array([0.77, -0.38, 0.78]) ]
    # xy_array = np.array([1, 1, 0])
    xy_array = np.array([-1, -1, 0])
    z_array = np.asarray([ 0, 0, -1 ])
    # z_array = np.asarray([ 0, 0, 1 ])
    targetPosition = np.array([0.77, -0.58, 0.78])
    epsilon = 100
    # epsilon = 0.005
    k_rep = 0.01
    k = 0.01

    # stop rule:
    # stop_rule_condition = np.linalg.norm( xyz_updated - xyz_prev)
    # while stop_rule_condition > epsilon or all(xyz_updated == 0):
    while True:

        pose_updated = np.zeros(shape=( (length_plan2-1)*7 ))

        print(f'num_left: {len(iter_points)}')

        # potential field updates
        for i in range(len(iter_points)):

            U_potential_gradient = 0

            for j in obstaclePositions:
                eePosition = iter_points[i][0]

                # get magnitude of force
                ro_b = math.dist(eePosition, j)
                # derivative_ro_b = np.asarray([ (eePosition[0] - j[0])/ro_b , (eePosition[1] - j[1])/ro_b , (eePosition[2] - j[2])/ro_b  ])
                # U_potential_gradient_rep = k_rep * (1/ro_b) * (1/(ro_b**2)) * derivative_ro_b
                # U_potential_gradient_rep = k_rep * (1/ro_b) * np.array([1, 1, 1])
                U_potential_gradient_rep = k_rep * (1/(ro_b**3))

                # get cylinder force in x y slots
                # dir_c = (xy_array*(eePosition - j)) / np.linalg.norm(xy_array*(eePosition - j))
                # dir_c = xy_array*((eePosition - j) / np.linalg.norm((eePosition - j)))

                # dir_target_c = (xy_array*(eePosition - targetPosition)) / np.linalg.norm(xy_array*(eePosition - targetPosition))
                # dir_c = dir_c - dir_target_c

                # U_potential_gradient_rep_cylinder = U_potential_gradient_rep * dir_c

                # get spherical force in z slot
                # dir_s = z_array*((eePosition - j) / np.linalg.norm((eePosition - j)))

                # dir_target_s = z_array*((eePosition - targetPosition) / np.linalg.norm(z_array*(eePosition - targetPosition)))
                # dir_s = dir_s - dir_target_s

                # U_potential_gradient_rep_sphere = U_potential_gradient_rep * dir_s
                #
                # # get psi
                # psi = np.linalg.norm( np.cross( (j - eePosition) , (targetPosition - eePosition) ) ) / np.linalg.norm( (targetPosition - eePosition) )

                k_attr = 1.0
                U_potential_gradient_attr = k_attr * (eePosition - targetPosition)
                # U_potential_gradient_attr = k_attr * (targetPosition - eePosition)
                # U_potential_gradient_attr = 0.5*k_attr * (eePosition - targetPosition)**2

                # add them to the total
                # U_potential_gradient += (U_potential_gradient_rep_cylinder + U_potential_gradient_rep_sphere) * psi
                # U_potential_gradient += (U_potential_gradient_rep_cylinder + U_potential_gradient_rep_sphere + U_potential_gradient_attr) * psi
                U_potential_gradient += (U_potential_gradient_rep + U_potential_gradient_attr)


            # U_potential_gradient *= xi

            # print(f'Gradient value: {U_potential_gradient}')
            # print('='*30)
            iter_points[i][0] -= k * U_potential_gradient

            total_plan[iter_points[i][1]][0] = copy.deepcopy(iter_points[i][0])

        # stop rule and list changing
        to_remove = []
        for i in range(len(total_plan) - 1):

            if np.linalg.norm(total_plan[i + 1][0] - total_plan[i][0]) <= epsilon:
                to_remove.append(i)

        for i in to_remove:
            for j in range(len(iter_points)):
                if i == iter_points[j][1]:
                    del iter_points[j]

                    break


        if len(iter_points) == 1:
            break


        # adding midpoints

        # get new previous points
        iter_points_prev = copy.deepcopy(iter_points)



    # modify plan if moveit can't move there: use cartesian path
    # Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints.
    # Configurations are computed for every eef_step meters; The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resultingpath.
    # The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory.
    waypoints = []
    wpose = group.get_current_pose().pose

    print('pose_updated')
    print(pose_updated)

    waypoints.append(copy.deepcopy(wpose))

    for i in range(len(total_plan)):
        wpose.position.x = total_plan[i][0][0]
        wpose.position.y = total_plan[i][0][1]
        wpose.position.z = total_plan[i][0][2]
        wpose.orientation.x = waypoints_orientation[i][0]
        wpose.orientation.y = waypoints_orientation[i][1]
        wpose.orientation.z = waypoints_orientation[i][2]
        wpose.orientation.w = waypoints_orientation[i][3]

        waypoints.append(copy.deepcopy(wpose))


    # for i in range(0,(length_plan2-2)*7,7):
    #     wpose.position.x = pose_updated[i]
    #     wpose.position.y = pose_updated[i+1]
    #     wpose.position.z = pose_updated[i+2]
    #     wpose.orientation.x = pose_updated[i+3]
    #     wpose.orientation.y = pose_updated[i+4]
    #     wpose.orientation.z = pose_updated[i+5]
    #     wpose.orientation.w = pose_updated[i+6]
    #     waypoints.append(copy.deepcopy(wpose))
    # append target position
    wpose.position.x = 0.77
    wpose.position.y = -0.58
    wpose.position.z = 0.75-0.78
    wpose.orientation.x = quaternion[0]
    wpose.orientation.y = quaternion[1]
    wpose.orientation.z = quaternion[2]
    wpose.orientation.w = quaternion[3]
    waypoints.append(copy.deepcopy(wpose))
    print('waypoints')
    print(waypoints)

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan2, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
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

    group.execute(plan2, wait=True)

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
    main()
