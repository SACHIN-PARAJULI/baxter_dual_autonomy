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
    # print('waypoints')
    # print(waypoints)

    (plan2, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # print('plan2')
    # print(plan2)
    # print('fraction')
    # print(fraction)

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
    # print('plan2')
    # print(plan2)
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
    # Q_update = np.zeros(shape=( length_plan2, dof_robot ))
    # print('Q_update')
    # print(Q_update)


    # Q_prev = np.zeros(shape=( length_plan2, dof_robot ))
    # load positions in plan
    # for i in range( length_plan2 ):
    #     Q_prev[i] = plan2.joint_trajectory.points[i].positions
    # print('Q_prev')
    # print(Q_prev)

    # plan2_original = copy.deepcopy(plan2)
    #
    # xyz_updated = np.zeros(shape=( (length_plan2-1)*3 ))
    # xyz_prev = np.zeros(shape=( (length_plan2-1)*3 ))
    # # print('xyz_updated')
    # # print(xyz_updated)
    #
    # pose_updated = np.zeros(shape=( (length_plan2-1)*7 ))

    length_plan2 = len(plan2.joint_trajectory.points)
    print('length plan2')
    print(length_plan2)

    # plan2_original = copy.deepcopy(plan2)
    #
    xyz_updated = np.zeros(shape=( (length_plan2)*3 ))
    xyz_prev = copy.deepcopy(xyz_updated)

    pose_updated = np.zeros(shape=( (length_plan2)*7 ))
    pose_prev = copy.deepcopy(pose_updated)

    # stop_rule_counter = np.inf
    # epsilon = 100
    counter = 0
    # np.set_printoptions(threshold=sys.maxsize)

    # stop rule:
    # norm( Q_update - Q_prev) <= epsilon
    # Q has all q positions from plan
    # stop_rule_condition = np.linalg.norm( xyz_updated - xyz_prev)
    # stop_rule_condition = 1000
    # while stop_rule_condition > epsilon or all(xyz_updated == 0):
    # while stop_rule_counter < 2:
    while True:
    # while np.linalg.norm( Q_update - Q_prev) > epsilon:
    # if not np.linalg.norm( Q_update - Q_prev) <= epsilon:

        # stop_rule_counter = 0

        # for i in range( length_plan2 ):
        #     Q_prev[i] = plan2.joint_trajectory.points[i].positions
        # print('Q_prev')
        # print(Q_prev)
        #
        # print( Q_update - Q_prev )
        # print( np.linalg.norm( Q_update - Q_prev) )
        # print( np.linalg.norm( Q_update - Q_prev) <= epsilon )
        # print('Q_update')
        # print(Q_update)
        # print('Q_prev')
        # print(Q_prev)

        length_plan2 = len(plan2.joint_trajectory.points)

        # plan2_original = copy.deepcopy(plan2)
        #
        xyz_prev = copy.deepcopy(xyz_updated)

        # # xyz_prev = np.zeros(shape=( (length_plan2-1)*3 ))
        # # print('xyz_updated')
        # # print(xyz_updated)
        #
        pose_prev = copy.deepcopy(pose_updated)
        # pose_updated = np.zeros(shape=( (length_plan2-1)*7 ))


        epsilon = 0.65
        index = 1
        index_modified = 1
        midpoints_indexes = []
        # while index < 1:
        # if stop_rule_counter > 0:
        if counter > 0:

            # wpose.position.x = 0.77
            # wpose.position.y = -0.58
            # wpose.position.z = 0.75-0.78
            # wpose.orientation.x = quaternion[0]
            # wpose.orientation.y = quaternion[1]
            # wpose.orientation.z = quaternion[2]
            # wpose.orientation.w = quaternion[3]

            pose_prev[-7] = 0.77
            pose_prev[-6] = -0.58
            pose_prev[-5] = 0.75-0.78
            pose_prev[-4] = quaternion[0]
            pose_prev[-3] = quaternion[1]
            pose_prev[-2] = quaternion[2]
            pose_prev[-1] = quaternion[3]
            print('pose_prev')
            print(pose_prev)

            xyz_prev[-3] = 0.77
            xyz_prev[-2] = -0.58
            xyz_prev[-1] = 0.75-0.78
            print('xyz_prev')
            print(xyz_prev)

            print('length plan2')
            print(length_plan2)
            print(len(xyz_prev))

            min_len = min(length_plan2,len(xyz_prev)/3)
            print('min_len')
            print(min_len)

            midpoints_indexes = []

            while index < min_len-1:

                # get point next and point current values
                joints_current = plan2.joint_trajectory.points[index].positions
                # joints_next = plan2.joint_trajectory.points[index+1].positions
                pose_current = kdl_kin.forward(joints_current)
                point_current = np.asarray([ pose_current.item((0, 3)), pose_current.item((1, 3)), pose_current.item((2, 3)) ])
                # pose_next = kdl_kin.forward(joints_next)
                # point_next = np.asarray([ pose_next.item((0, 3)), pose_next.item((1, 3)), pose_next.item((2, 3)) ])
                point_previous = xyz_prev[index*3:index*3+3]
                # point_previous = pose_prev[index*7:index*7+3]

                print('points')
                print(point_current)
                print(point_previous)

                # stop rule check
                # if norm of point next and point current greater epsilon then add a midpoint
                print('norm')
                print(np.linalg.norm( point_previous - point_current ))
                if np.linalg.norm( point_previous - point_current ) > epsilon:
                    midpoint = (point_previous + point_current)/2
                    print('midpoint')
                    print(midpoint)

                    # midpoint_pose

                    # print('pose_current')
                    # print(pose_current)
                    pose_midpoint = pose_current
                    pose_midpoint[0,3] = midpoint[0]
                    pose_midpoint[1,3] = midpoint[1]
                    pose_midpoint[2,3] = midpoint[2]
                    # print('pose_midpoint')
                    # print(pose_midpoint)

                    joints_midpoint = kdl_kin.inverse(pose_midpoint)

                    # l.insert(2, 100)
                    # [0, 1, 2]
                    # [0, 1, 100, 2]

                    q_points_list = list(plan2.joint_trajectory.points)
                    # print('q_points_list')
                    # print(q_points_list)
                    # q_points_list.insert(2, 100)

                    q_points_list_index = q_points_list[index_modified]
                    # print('q_points_list_index')
                    # print(q_points_list_index)
                    q_positions_list = list(plan2.joint_trajectory.points[index_modified].positions)
                    for i in range( len(q_positions_list) ):
                        q_positions_list[i] = joints_midpoint[i].item()

                    q_points_list_index.positions = q_positions_list
                    # print('q_points_list_index mod')
                    # print(q_points_list_index)
                    # plan2.joint_trajectory.points[index_modified].positions = tuple(q_positions_list)
                    q_points_list.insert(index_modified, q_points_list_index)

                    plan2.joint_trajectory.points = tuple(q_points_list)

                    # print('plan2 modified')
                    # print(plan2)

                    # we want to recalculate U_potential_gradient_rep ONLY if it is for a newly added midpoint
                    # save current index (variable index_modified) to an array
                    midpoints_indexes.append(index_modified)

                    index_modified = index_modified + 1



                index = index + 1
                index_modified = index_modified + 1

            if index == index_modified:
                # stop while loop
                break

            print('midpoints_indexes')
            print(midpoints_indexes)

        length_plan2 = len(plan2.joint_trajectory.points)
        print('length plan2')
        print(length_plan2)

        # plan2_original = copy.deepcopy(plan2)
        #
        xyz_prev = copy.deepcopy(xyz_updated)
        pose_prev = copy.deepcopy(pose_updated)

        # # xyz_prev = np.zeros(shape=( (length_plan2-1)*3 ))
        # # print('xyz_updated')
        # # print(xyz_updated)
        #
        pose_updated = np.zeros(shape=( (length_plan2)*7 ))

        xyz_updated = np.zeros(shape=( (length_plan2)*3 ))

        # pose_updated = np.resize((length_plan2)*7 )
        # xyz_updated = np.resize((length_plan2)*3 )

        index = 0
        index_midpoint = 0
        # while index < 1:
        while index < length_plan2-1:
        # while (index < length_plan2-1 and counter == 0) or (index_midpoint < len(midpoints_indexes) and counter > 0):

            # if counter > 0:
            #     index = midpoints_indexes[index_midpoint]

            print('index')
            print(index)

            # get current and next part of moveit plan
            plan2_part = copy.deepcopy(plan2)
            # plan2_part = plan2
            # indexes = [index, index+1]
            # plan2_part.joint_trajectory.points = [plan2_part.joint_trajectory.points[x] for x in indexes]
            # # joint_positions_planned_next = np.asarray(plan2_part.joint_trajectory.points[0].positions)
            # # print ('plan2_part')
            # # print (plan2_part)
            # joint_positions_planned_next = np.asarray(plan2_part.joint_trajectory.points[-1].positions)
            joint_positions_planned_next = np.asarray(plan2_part.joint_trajectory.points[index].positions)
            # print ('joint_positions_planned_next')
            # print (joint_positions_planned_next)

            # zeta: attractive potential gain
            # zeta = 0.1
            # ro_g: relative Cartesian distance between the end effector of the arm and the target

            # calculate joint to pose values
            pose = kdl_kin.forward(joint_positions_planned_next)
            eePosition_moveit_plan_next = np.asarray([ pose.item((0, 3)), pose.item((1, 3)), pose.item((2, 3)) ])
            # print ('eePosition_moveit_plan_next')
            # print (eePosition_moveit_plan_next)

            # print('pose')
            # print(pose)
            quat_updated = quaternion_from_matrix(pose)
            # print('quat_updated')
            # print(quat_updated)
            pose_index = np.asarray([ pose.item((0, 3)), pose.item((1, 3)), pose.item((2, 3)), quat_updated[0], quat_updated[1], quat_updated[2], quat_updated[3] ])
            # pose_updated[index*7:index*7+7] = pose_index
            pose_updated[index*7+3:index*7+7] = pose_index[3:7]
            # print('pose_index')
            # print(pose_index)
            # print(' ')
            # print('pose_updated')
            # print(pose_updated)
            # print(' ')

            if (index < length_plan2-1 and counter == 0) or (index_midpoint < len(midpoints_indexes) and counter > 0):

                if counter > 0:
                    index_prev = index
                    index = midpoints_indexes[index_midpoint]

                    print('index midpoint')
                    print(index)

                eePosition = eePosition_moveit_plan_next
                # targetPosition = np.asarray([0.77, 0.0, 0.78])
                # mug4_pose=Pose(position=Point(x=0.77, y=-0.58, z=0.78)),
                targetPosition = np.asarray([0.77, -0.58, eePosition[2]])
                # targetPosition = np.asarray([0.77, 0.0, eePosition[2]])

                # ro_g = math.dist(eePosition, targetPosition)
                # print('ro_g')
                # print(ro_g)
                # U_potential_gradient_att = zeta * ro_g
                # k_rep: repulsive potential gain
                # k_rep = 0.01
                k_rep = 0.01
                k = 0.01

                # ro_0: max range of the repulsive potential field
                # ro_0 = 0.075
                # ro_0 = 0.3
                targetPosition_mod = np.asarray([0.77, -0.58, eePosition[2]])
                # targetPosition_mod = np.asarray([0.77, 0.0, 0.78])
                # ro_g_mod = math.dist(eePosition, targetPosition_mod)
                # print('ro_g_mod')
                # print(ro_g_mod)
                # ro_b: minimum distance between the robot's body and the obstacle/ measures the workspace distance between the robot and the obstacle
                   # scissors_pose=Pose(position=Point(x=0.55, y=-0.1, z=0.35)),
                   # clock_pose=Pose(position=Point(x=0.45, y=0, z=0.4)),
                   # bowl_pose=Pose(position=Point(x=0.55, y=0.2, z=0.4)),
                   # tennis_ball_pose=Pose(position=Point(x=0.6, y=0.15, z=0.35))
                obstaclePosition_1 = np.asarray([0.77, 0.22, eePosition[2]])
                # obstaclePosition_1 = np.asarray([0.75, -0.1, 0.78])
                ro_b_1 = math.dist(eePosition, obstaclePosition_1)
                # print(ro_b_1)
                # if ro_b_1 <= ro_0:
                # # if ro_b_1 <= ro_g_mod:
                #     # U_potential_gradient_rep_1 = k_rep * (1/ro_b_1 - 1/ro_0) * (1/(ro_b_1**2))
                #     derivative_ro_b_1 = np.asarray([ (eePosition[0] - obstaclePosition_1[0])/ro_b_1 , (eePosition[1] - obstaclePosition_1[1])/ro_b_1 , 0 ])
                #     # derivative_ro_b_1 = np.asarray([ (eePosition[0] - obstaclePosition_1[0])/ro_b_1 , (eePosition[1] - obstaclePosition_1[1])/ro_b_1 , (eePosition[2] - obstaclePosition_1[2])/ro_b_1 ])
                #     print('derivative_ro_b_1')
                #     print(derivative_ro_b_1)
                #     # U_potential_gradient_rep_1 = k_rep * (1/ro_b_1 - 1/ro_g_mod) * (1/(ro_b_1**2)) * derivative_ro_b_1
                #     U_potential_gradient_rep_1 = k_rep * (1/ro_b_1 - 1/ro_0) * (1/(ro_b_1**2)) * derivative_ro_b_1
                # else:
                #     U_potential_gradient_rep_1 = 0.0
                derivative_ro_b_1 = np.asarray([ (eePosition[0] - obstaclePosition_1[0])/ro_b_1 , (eePosition[1] - obstaclePosition_1[1])/ro_b_1 , 0 ])
                U_potential_gradient_rep_1 = k_rep * (1/ro_b_1) * (1/(ro_b_1**2)) * derivative_ro_b_1
                obstaclePosition_2 = np.asarray([0.77, 0.02, eePosition[2]])
                # obstaclePosition_2 = np.asarray([0.65, 0, 0.78])
                ro_b_2 = math.dist(eePosition, obstaclePosition_2)
                # print(ro_b_2)
                # if ro_b_2 <= ro_0:
                # # if ro_b_2 <= ro_g_mod:
                #     # U_potential_gradient_rep_2 = k_rep * (1/ro_b_2 - 1/ro_0) * (1/(ro_b_2**2))
                #     derivative_ro_b_2 = np.asarray([ (eePosition[0] - obstaclePosition_2[0])/ro_b_2 , (eePosition[1] - obstaclePosition_2[1])/ro_b_2 , 0 ])
                #     # derivative_ro_b_2 = np.asarray([ (eePosition[0] - obstaclePosition_2[0])/ro_b_2 , (eePosition[1] - obstaclePosition_2[1])/ro_b_2 , (eePosition[2] - obstaclePosition_2[2])/ro_b_2 ])
                #     # U_potential_gradient_rep_2 = k_rep * (1/ro_b_2 - 1/ro_g_mod) * (1/(ro_b_2**2)) * derivative_ro_b_2
                #     U_potential_gradient_rep_2 = k_rep * (1/ro_b_2 - 1/ro_0) * (1/(ro_b_2**2)) * derivative_ro_b_2
                # else:
                #     U_potential_gradient_rep_2 = 0.0
                derivative_ro_b_2 = np.asarray([ (eePosition[0] - obstaclePosition_2[0])/ro_b_2 , (eePosition[1] - obstaclePosition_2[1])/ro_b_2 , 0 ])
                U_potential_gradient_rep_2 = k_rep * (1/ro_b_2) * (1/(ro_b_2**2)) * derivative_ro_b_2
                obstaclePosition_3 = (0.77, -0.18, eePosition[2])
                # obstaclePosition_3 = (0.75, 0.2, 0.78)
                ro_b_3 = math.dist(eePosition, obstaclePosition_3)
                # print(ro_b_3)
                # if ro_b_3 <= ro_0:
                # # if ro_b_3 <= ro_g_mod:
                #     # U_potential_gradient_rep_3 = k_rep * (1/ro_b_3 - 1/ro_0) * (1/(ro_b_3**2))
                #     # U_potential_gradient_rep_3 = k_rep * (1/ro_b_3 - 1/ro_g_mod) * (1/(ro_b_3**2))
                #     derivative_ro_b_3 = np.asarray([ (eePosition[0] - obstaclePosition_3[0])/ro_b_3 , (eePosition[1] - obstaclePosition_3[1])/ro_b_3 , 0 ])
                #     # derivative_ro_b_3 = np.asarray([ (eePosition[0] - obstaclePosition_3[0])/ro_b_3 , (eePosition[1] - obstaclePosition_3[1])/ro_b_3 , (eePosition[2] - obstaclePosition_3[2])/ro_b_3 ])
                #     print('derivative_ro_b_3')
                #     print(derivative_ro_b_3)
                #     U_potential_gradient_rep_3 = k_rep * (1/ro_b_3 - 1/ro_0) * (1/(ro_b_3**2)) * derivative_ro_b_3
                #     # U_potential_gradient_rep_3 = k_rep * (1/ro_b_3 - 1/ro_g_mod) * (1/(ro_b_3**2)) * derivative_ro_b_3
                # else:
                #     U_potential_gradient_rep_3 = 0.0
                derivative_ro_b_3 = np.asarray([ (eePosition[0] - obstaclePosition_3[0])/ro_b_3 , (eePosition[1] - obstaclePosition_3[1])/ro_b_3 , 0 ])
                U_potential_gradient_rep_3 = k_rep * (1/ro_b_3) * (1/(ro_b_3**2)) * derivative_ro_b_3
                obstaclePosition_4 = (0.77, -0.38, eePosition[2])
                # obstaclePosition_4 = (0.8, 0.15, 0.78)
                ro_b_4 = math.dist(eePosition, obstaclePosition_4)
                # print(ro_b_4)
                # if ro_b_4 <= ro_0:
                # # if ro_b_4 <= ro_g_mod:
                #     # U_potential_gradient_rep_4 = k_rep * (1/ro_b_4 - 1/ro_0) * (1/(ro_b_4**2))
                #     # U_potential_gradient_rep_4 = k_rep * (1/ro_b_4 - 1/ro_g_mod) * (1/(ro_b_4**2))
                #     derivative_ro_b_4 = np.asarray([ (eePosition[0] - obstaclePosition_4[0])/ro_b_4 , (eePosition[1] - obstaclePosition_4[1])/ro_b_4 , 0 ])
                #     # derivative_ro_b_4 = np.asarray([ (eePosition[0] - obstaclePosition_4[0])/ro_b_4 , (eePosition[1] - obstaclePosition_4[1])/ro_b_4 , (eePosition[2] - obstaclePosition_4[2])/ro_b_4 ])
                #     print('derivative_ro_b_4')
                #     print(derivative_ro_b_4)
                #     # U_potential_gradient_rep_4 = k_rep * (1/ro_b_4 - 1/ro_g_mod) * (1/(ro_b_4**2)) * derivative_ro_b_4
                #     U_potential_gradient_rep_4 = k_rep * (1/ro_b_4 - 1/ro_0) * (1/(ro_b_4**2)) * derivative_ro_b_4
                # else:
                #     U_potential_gradient_rep_4 = 0.0
                derivative_ro_b_4 = np.asarray([ (eePosition[0] - obstaclePosition_4[0])/ro_b_4 , (eePosition[1] - obstaclePosition_4[1])/ro_b_4 , 0 ])
                U_potential_gradient_rep_4 = k_rep * (1/ro_b_4) * (1/(ro_b_4**2)) * derivative_ro_b_4
                # dir_target = (eePosition - targetPosition) / np.linalg.norm(eePosition - targetPosition)
                # print(dir_target)
                # # obstacle 1
                # dir_1 = (eePosition - obstaclePosition_1) / np.linalg.norm(eePosition - obstaclePosition_1)
                # # with respect to ee and target direction
                # dir_1 = dir_1 - dir_target
                # U_potential_gradient_rep_1_cylinder = U_potential_gradient_rep_1 * dir_1
                # # obstacle 2
                # dir_2 = (eePosition - obstaclePosition_2) / np.linalg.norm(eePosition - obstaclePosition_2)
                # # with respect to ee and target direction
                # dir_2 = dir_2 - dir_target
                # U_potential_gradient_rep_2_cylinder = U_potential_gradient_rep_2 * dir_2
                # # obstacle 3
                # dir_3 = (eePosition - obstaclePosition_3) / np.linalg.norm(eePosition - obstaclePosition_3)
                # # with respect to ee and target direction
                # dir_3 = dir_3 - dir_target
                # U_potential_gradient_rep_3_cylinder = U_potential_gradient_rep_3 * dir_3
                # # obstacle 4
                # dir_4 = (eePosition - obstaclePosition_4) / np.linalg.norm(eePosition - obstaclePosition_4)
                # # with respect to ee and target direction
                # dir_4 = dir_4 - dir_target
                # U_potential_gradient_rep_4_cylinder = U_potential_gradient_rep_4 * dir_4
                xy_array = np.asarray([ 1, 1, 0 ])
                # dir_target = xy_array*(eePosition - targetPosition) / np.linalg.norm(xy_array*(eePosition - targetPosition))
                # print(dir_target)
                # obstacle 1
                dir_1 = xy_array*(eePosition - obstaclePosition_1) / np.linalg.norm(xy_array*(eePosition - obstaclePosition_1))
                # with respect to ee and target direction
                # dir_1 = dir_1 - dir_target
                U_potential_gradient_rep_1_cylinder = U_potential_gradient_rep_1 * dir_1
                # obstacle 2
                dir_2 = xy_array*(eePosition - obstaclePosition_2) / np.linalg.norm(xy_array*(eePosition - obstaclePosition_2))
                # with respect to ee and target direction
                # dir_2 = dir_2 - dir_target
                U_potential_gradient_rep_2_cylinder = U_potential_gradient_rep_2 * dir_2
                # obstacle 3
                dir_3 = xy_array*(eePosition - obstaclePosition_3) / np.linalg.norm(xy_array*(eePosition - obstaclePosition_3))
                # with respect to ee and target direction
                # dir_3 = dir_3 - dir_target
                U_potential_gradient_rep_3_cylinder = U_potential_gradient_rep_3 * dir_3
                # obstacle 4
                dir_4 = xy_array*(eePosition - obstaclePosition_4) / np.linalg.norm(xy_array*(eePosition - obstaclePosition_4))
                # with respect to ee and target direction
                # dir_4 = dir_4 - dir_target
                U_potential_gradient_rep_4_cylinder = U_potential_gradient_rep_4 * dir_4

                # print("dir_1:{}, dir_2:{}, dir_3:{}, dir_4:{}\n".format(dir_1,dir_2,dir_3,dir_4))
                #
                # print("ro_b_1:{}, ro_b_2:{}, ro_b_3:{}, ro_b_4:{}\n".format(ro_b_1,ro_b_2,ro_b_3,ro_b_4))
                # # print("U_potential_gradient_rep_1:{}, U_potential_gradient_rep_2:{}, U_potential_gradient_rep_3:{}, U_potential_gradient_rep_4:{}\n".format(U_potential_gradient_rep_1,U_potential_gradient_rep_2,U_potential_gradient_rep_3,U_potential_gradient_rep_4))
                # print("U_potential_gradient_rep_1_cylinder:{}, U_potential_gradient_rep_2_cylinder:{}, U_potential_gradient_rep_3_cylinder:{}, U_potential_gradient_rep_4_cylinder:{}\n".format(U_potential_gradient_rep_1_cylinder,U_potential_gradient_rep_2_cylinder,U_potential_gradient_rep_3_cylinder,U_potential_gradient_rep_4_cylinder))
                #

                # calculation for sphere
                # targetPosition_mod = np.asarray([0.77, 0.0, eePosition[2]])
                targetPosition_mod = np.asarray([0.77, -0.58, 0.78])
                # ro_g_mod = math.dist(eePosition, targetPosition_mod)
                # print('ro_g_mod')
                # print(ro_g_mod)
                # ro_b: minimum distance between the robot's body and the obstacle/ measures the workspace distance between the robot and the obstacle
                   # scissors_pose=Pose(position=Point(x=0.55, y=-0.1, z=0.35)),
                   # clock_pose=Pose(position=Point(x=0.45, y=0, z=0.4)),
                   # bowl_pose=Pose(position=Point(x=0.55, y=0.2, z=0.4)),
                   # tennis_ball_pose=Pose(position=Point(x=0.6, y=0.15, z=0.35))
                # obstaclePosition_1 = np.asarray([0.75, -0.1, eePosition[2]])
                obstaclePosition_1 = np.asarray([0.77, 0.22, 0.78])
                ro_b_1 = math.dist(eePosition, obstaclePosition_1)
                # print(ro_b_1)
                # if ro_b_1 <= ro_0:
                # # if ro_b_1 <= ro_g_mod:
                #     # U_potential_gradient_rep_1 = k_rep * (1/ro_b_1 - 1/ro_0) * (1/(ro_b_1**2))
                #     derivative_ro_b_1 = np.asarray([ (eePosition[0] - obstaclePosition_1[0])/ro_b_1 , (eePosition[1] - obstaclePosition_1[1])/ro_b_1 , 0 ])
                #     # derivative_ro_b_1 = np.asarray([ (eePosition[0] - obstaclePosition_1[0])/ro_b_1 , (eePosition[1] - obstaclePosition_1[1])/ro_b_1 , (eePosition[2] - obstaclePosition_1[2])/ro_b_1 ])
                #     print('derivative_ro_b_1')
                #     print(derivative_ro_b_1)
                #     # U_potential_gradient_rep_1 = k_rep * (1/ro_b_1 - 1/ro_g_mod) * (1/(ro_b_1**2)) * derivative_ro_b_1
                #     U_potential_gradient_rep_1 = k_rep * (1/ro_b_1 - 1/ro_0) * (1/(ro_b_1**2)) * derivative_ro_b_1
                # else:
                #     U_potential_gradient_rep_1 = 0.0
                # derivative_ro_b_1 = np.asarray([ (eePosition[0] - obstaclePosition_1[0])/ro_b_1 , (eePosition[1] - obstaclePosition_1[1])/ro_b_1 , 0 ])
                derivative_ro_b_1 = np.asarray([ (eePosition[0] - obstaclePosition_1[0])/ro_b_1 , (eePosition[1] - obstaclePosition_1[1])/ro_b_1 , (eePosition[2] - obstaclePosition_1[2])/ro_b_1 ])
                U_potential_gradient_rep_1 = k_rep * (1/ro_b_1) * (1/(ro_b_1**2)) * derivative_ro_b_1
                # obstaclePosition_2 = np.asarray([0.65, 0, eePosition[2]])
                obstaclePosition_2 = np.asarray([0.77, 0.02, 0.78])
                ro_b_2 = math.dist(eePosition, obstaclePosition_2)
                # print(ro_b_2)
                # if ro_b_2 <= ro_0:
                # # if ro_b_2 <= ro_g_mod:
                #     # U_potential_gradient_rep_2 = k_rep * (1/ro_b_2 - 1/ro_0) * (1/(ro_b_2**2))
                #     derivative_ro_b_2 = np.asarray([ (eePosition[0] - obstaclePosition_2[0])/ro_b_2 , (eePosition[1] - obstaclePosition_2[1])/ro_b_2 , 0 ])
                #     # derivative_ro_b_2 = np.asarray([ (eePosition[0] - obstaclePosition_2[0])/ro_b_2 , (eePosition[1] - obstaclePosition_2[1])/ro_b_2 , (eePosition[2] - obstaclePosition_2[2])/ro_b_2 ])
                #     # U_potential_gradient_rep_2 = k_rep * (1/ro_b_2 - 1/ro_g_mod) * (1/(ro_b_2**2)) * derivative_ro_b_2
                #     U_potential_gradient_rep_2 = k_rep * (1/ro_b_2 - 1/ro_0) * (1/(ro_b_2**2)) * derivative_ro_b_2
                # else:
                #     U_potential_gradient_rep_2 = 0.0
                # derivative_ro_b_2 = np.asarray([ (eePosition[0] - obstaclePosition_2[0])/ro_b_2 , (eePosition[1] - obstaclePosition_2[1])/ro_b_2 , 0 ])
                derivative_ro_b_2 = np.asarray([ (eePosition[0] - obstaclePosition_2[0])/ro_b_2 , (eePosition[1] - obstaclePosition_2[1])/ro_b_2 , (eePosition[2] - obstaclePosition_2[2])/ro_b_2 ])
                U_potential_gradient_rep_2 = k_rep * (1/ro_b_2) * (1/(ro_b_2**2)) * derivative_ro_b_2
                # obstaclePosition_3 = (0.75, 0.2, eePosition[2])
                obstaclePosition_3 = (0.77, -0.18, 0.78)
                ro_b_3 = math.dist(eePosition, obstaclePosition_3)
                # print(ro_b_3)
                # if ro_b_3 <= ro_0:
                # # if ro_b_3 <= ro_g_mod:
                #     # U_potential_gradient_rep_3 = k_rep * (1/ro_b_3 - 1/ro_0) * (1/(ro_b_3**2))
                #     # U_potential_gradient_rep_3 = k_rep * (1/ro_b_3 - 1/ro_g_mod) * (1/(ro_b_3**2))
                #     derivative_ro_b_3 = np.asarray([ (eePosition[0] - obstaclePosition_3[0])/ro_b_3 , (eePosition[1] - obstaclePosition_3[1])/ro_b_3 , 0 ])
                #     # derivative_ro_b_3 = np.asarray([ (eePosition[0] - obstaclePosition_3[0])/ro_b_3 , (eePosition[1] - obstaclePosition_3[1])/ro_b_3 , (eePosition[2] - obstaclePosition_3[2])/ro_b_3 ])
                #     print('derivative_ro_b_3')
                #     print(derivative_ro_b_3)
                #     U_potential_gradient_rep_3 = k_rep * (1/ro_b_3 - 1/ro_0) * (1/(ro_b_3**2)) * derivative_ro_b_3
                #     # U_potential_gradient_rep_3 = k_rep * (1/ro_b_3 - 1/ro_g_mod) * (1/(ro_b_3**2)) * derivative_ro_b_3
                # else:
                #     U_potential_gradient_rep_3 = 0.0
                # derivative_ro_b_3 = np.asarray([ (eePosition[0] - obstaclePosition_3[0])/ro_b_3 , (eePosition[1] - obstaclePosition_3[1])/ro_b_3 , 0 ])
                derivative_ro_b_3 = np.asarray([ (eePosition[0] - obstaclePosition_3[0])/ro_b_3 , (eePosition[1] - obstaclePosition_3[1])/ro_b_3 , (eePosition[2] - obstaclePosition_3[2])/ro_b_3 ])
                U_potential_gradient_rep_3 = k_rep * (1/ro_b_3) * (1/(ro_b_3**2)) * derivative_ro_b_3
                # obstaclePosition_4 = (0.8, 0.15, eePosition[2])
                obstaclePosition_4 = (0.77, -0.38, 0.78)
                # ro_b_4 = math.dist(eePosition, obstaclePosition_4)
                # print(ro_b_4)
                # if ro_b_4 <= ro_0:
                # # if ro_b_4 <= ro_g_mod:
                #     # U_potential_gradient_rep_4 = k_rep * (1/ro_b_4 - 1/ro_0) * (1/(ro_b_4**2))
                #     # U_potential_gradient_rep_4 = k_rep * (1/ro_b_4 - 1/ro_g_mod) * (1/(ro_b_4**2))
                #     derivative_ro_b_4 = np.asarray([ (eePosition[0] - obstaclePosition_4[0])/ro_b_4 , (eePosition[1] - obstaclePosition_4[1])/ro_b_4 , 0 ])
                #     # derivative_ro_b_4 = np.asarray([ (eePosition[0] - obstaclePosition_4[0])/ro_b_4 , (eePosition[1] - obstaclePosition_4[1])/ro_b_4 , (eePosition[2] - obstaclePosition_4[2])/ro_b_4 ])
                #     print('derivative_ro_b_4')
                #     print(derivative_ro_b_4)
                #     # U_potential_gradient_rep_4 = k_rep * (1/ro_b_4 - 1/ro_g_mod) * (1/(ro_b_4**2)) * derivative_ro_b_4
                #     U_potential_gradient_rep_4 = k_rep * (1/ro_b_4 - 1/ro_0) * (1/(ro_b_4**2)) * derivative_ro_b_4
                # else:
                #     U_potential_gradient_rep_4 = 0.0
                # derivative_ro_b_4 = np.asarray([ (eePosition[0] - obstaclePosition_4[0])/ro_b_4 , (eePosition[1] - obstaclePosition_4[1])/ro_b_4 , 0 ])
                derivative_ro_b_4 = np.asarray([ (eePosition[0] - obstaclePosition_4[0])/ro_b_4 , (eePosition[1] - obstaclePosition_4[1])/ro_b_4 , (eePosition[2] - obstaclePosition_4[2])/ro_b_4 ])
                U_potential_gradient_rep_4 = k_rep * (1/ro_b_4) * (1/(ro_b_4**2)) * derivative_ro_b_4
                z_array = np.asarray([ 0, 0, -1 ])
                # dir_target = z_array*(eePosition - targetPosition_mod) / np.linalg.norm(z_array*(eePosition - targetPosition_mod))
                # print(dir_target)
                # obstacle 1
                dir_1 = z_array*(eePosition - obstaclePosition_1) / np.linalg.norm(z_array*(eePosition - obstaclePosition_1))
                # with respect to ee and target direction
                # dir_1 = dir_1 - dir_target
                U_potential_gradient_rep_1_sphere = U_potential_gradient_rep_1 * dir_1
                # obstacle 2
                dir_2 = z_array*(eePosition - obstaclePosition_2) / np.linalg.norm(z_array*(eePosition - obstaclePosition_2))
                # with respect to ee and target direction
                # dir_2 = dir_2 - dir_target
                U_potential_gradient_rep_2_sphere = U_potential_gradient_rep_2 * dir_2
                # obstacle 3
                dir_3 = z_array*(eePosition - obstaclePosition_3) / np.linalg.norm(z_array*(eePosition - obstaclePosition_3))
                # with respect to ee and target direction
                # dir_3 = dir_3 - dir_target
                U_potential_gradient_rep_3_sphere = U_potential_gradient_rep_3 * dir_3
                # obstacle 4
                dir_4 = z_array*(eePosition - obstaclePosition_4) / np.linalg.norm(z_array*(eePosition - obstaclePosition_4))
                # with respect to ee and target direction
                # dir_4 = dir_4 - dir_target
                U_potential_gradient_rep_4_sphere = U_potential_gradient_rep_4 * dir_4
                # print("dir_1:{}, dir_2:{}, dir_3:{}, dir_4:{}\n".format(dir_1,dir_2,dir_3,dir_4))
                #
                # print("ro_b_1:{}, ro_b_2:{}, ro_b_3:{}, ro_b_4:{}\n".format(ro_b_1,ro_b_2,ro_b_3,ro_b_4))
                # # print("U_potential_gradient_rep_1:{}, U_potential_gradient_rep_2:{}, U_potential_gradient_rep_3:{}, U_potential_gradient_rep_4:{}\n".format(U_potential_gradient_rep_1,U_potential_gradient_rep_2,U_potential_gradient_rep_3,U_potential_gradient_rep_4))
                # print("U_potential_gradient_rep_1_sphere:{}, U_potential_gradient_rep_2_sphere:{}, U_potential_gradient_rep_3_sphere:{}, U_potential_gradient_rep_4_sphere:{}\n".format(U_potential_gradient_rep_1_sphere,U_potential_gradient_rep_2_sphere,U_potential_gradient_rep_3_sphere,U_potential_gradient_rep_4_sphere))
                #



                # # print("ro_b_1:{}, ro_b_2:{}, ro_b_3:{}, ro_b_4:{}\n".format(ro_b_1,ro_b_2,ro_b_3,ro_b_4))
                # # print("U_potential_gradient_rep_1:{}, U_potential_gradient_rep_2:{}, U_potential_gradient_rep_3:{}, U_potential_gradient_rep_4:{}\n".format(U_potential_gradient_rep_1,U_potential_gradient_rep_2,U_potential_gradient_rep_3,U_potential_gradient_rep_4))
                # # U_potential_gradient_rep = U_potential_gradient_rep_1 + U_potential_gradient_rep_2 + U_potential_gradient_rep_3 + U_potential_gradient_rep_4
                # # U_potential_gradient_rep = U_potential_gradient_rep_1
                # U_potential_gradient_rep = U_potential_gradient_rep_1 + U_potential_gradient_rep_2
                # # U_potential_gradient = U_potential_gradient_att + U_potential_gradient_rep
                # # print(U_potential_gradient_att)
                # print(U_potential_gradient_rep)
                # U_potential_gradient = U_potential_gradient_rep
                # Force = -U_potential_gradient
                # print('Force')
                # print(Force)
                # k=0.001
                # k = 0.01

                # calculate normalized orthogonal minimum to find shortest distance from object to distance of the end effector and the target
                # for each object: create a vector
                psi_1 = np.linalg.norm( np.cross( (obstaclePosition_1-eePosition) , (targetPosition-eePosition) ) ) / np.linalg.norm( (targetPosition-eePosition) )
                # print('psi_1')
                # print(psi_1)
                psi_2 = np.linalg.norm( np.cross( (obstaclePosition_2-eePosition) , (targetPosition-eePosition) ) ) / np.linalg.norm( (targetPosition-eePosition) )
                # print('psi_2')
                # print(psi_2)
                psi_3 = np.linalg.norm( np.cross( (obstaclePosition_3-eePosition) , (targetPosition-eePosition) ) ) / np.linalg.norm( (targetPosition-eePosition) )
                # print('psi_3')
                # print(psi_3)
                psi_4 = np.linalg.norm( np.cross( (obstaclePosition_4-eePosition) , (targetPosition-eePosition) ) ) / np.linalg.norm( (targetPosition-eePosition) )
                # print('psi_4')
                # print(psi_4)


                # calculate U_potential_gradient
                # U_potential_gradient = xi * ( U_potential_gradient_rep_1*psi_1 + U_potential_gradient_rep_2*psi_2 + U_potential_gradient_rep_3*psi_3 + U_potential_gradient_rep_4*psi_4 )
                # U_potential_gradient = U_potential_gradient_rep_1 + U_potential_gradient_rep_2 + U_potential_gradient_rep_3 + U_potential_gradient_rep_4
                U_potential_gradient = xi * ( (U_potential_gradient_rep_1_cylinder+U_potential_gradient_rep_1_sphere)*psi_1 + (U_potential_gradient_rep_2_cylinder+U_potential_gradient_rep_2_sphere)*psi_2 + (U_potential_gradient_rep_3_cylinder+U_potential_gradient_rep_3_sphere)*psi_3 + (U_potential_gradient_rep_4_cylinder+U_potential_gradient_rep_4_sphere)*psi_4 )
                # print('U_potential_gradient')
                # print(U_potential_gradient)


                eePosition_moveit_plan_next_updated = eePosition_moveit_plan_next - k*U_potential_gradient

                q_positions_list = list(plan2.joint_trajectory.points[index].positions)
                q_positions_list = plan2_part.joint_trajectory.points[index].positions
                plan2.joint_trajectory.points[index].positions = tuple(q_positions_list)
            # print('eePosition_moveit_plan_next_updated')
            # print(eePosition_moveit_plan_next_updated)

            # converting pose back to joint values not needed since we will input this to compute moveit cartesian path
            # convert pose back to joint angles
            # pose = kdl_kin.forward(joint_positions_planned_next)
            # print (pose)
            # pose_updated = copy.deepcopy(pose)
            # pose_updated[0, 3] = eePosition_moveit_plan_next_updated[0]
            # pose_updated[1, 3] = eePosition_moveit_plan_next_updated[1]
            # pose_updated[2, 3] = eePosition_moveit_plan_next_updated[2]
            # print (pose_updated)
            #
            # eeJoints_moveit_plan_next_updated = kdl_kin.inverse(pose_updated)
            # if eeJoints_moveit_plan_next_updated is not None:
            #     print ('eeJoints_moveit_plan_next_updated')
            #     print (eeJoints_moveit_plan_next_updated)
            # else:
            #     # take the moveit values instead
            #     eeJoints_moveit_plan_next_updated = joint_positions_planned_next
            #     print('update with potential field failed')
            #
            # q_positions_list = list(plan2_part.joint_trajectory.points[-1].positions)
            # print( len(q_positions_list) )
            # for i in range( len(q_positions_list) ):
            #     q_positions_list[i] = eeJoints_moveit_plan_next_updated[i].item()
            #
            # plan2_part.joint_trajectory.points[-1].positions = tuple(q_positions_list)
            # print ('plan2_part')
            # print (plan2_part)

                if counter == 0:
                    index = index + 1
                else:
                    index = index_prev
                    index_midpoint = index_midpoint + 1

            else:
                eePosition_moveit_plan_next_updated = eePosition_moveit_plan_next
                index = index + 1

            # append pose xyz values in vector
            xyz_updated[index*3:index*3+3] = eePosition_moveit_plan_next_updated
            pose_updated[index*7:index*7+3] = eePosition_moveit_plan_next_updated
            # print(' ')
            # print('pose_updated')
            # print(pose_updated)
            # print(' ')


            # group.execute(plan2_part, wait=True)


            # plan2_part = copy.deepcopy(plan2)
            # indexes = [index, index+1]
            # plan2_part.joint_trajectory.points = [plan2_part.joint_trajectory.points[x] for x in indexes]

            # update the plan
            # q_positions_list = list(plan2.joint_trajectory.points[index+1].positions)
            # q_positions_list = plan2_part.joint_trajectory.points[-1].positions
            # plan2.joint_trajectory.points[index+1].positions = tuple(q_positions_list)


            # index = index + 1
            # index_midpoint = index_midpoint + 1

        print('xyz_updated')
        print(xyz_updated)

        print('plan2')
        print(plan2)

        # stop rule:
        # norm( Q_update - Q_prev) <= epsilon
        # Q has all q positions from plan
        # for i in range( length_plan2 ):
        #     Q_update[i] = plan2.joint_trajectory.points[i].positions
        # print('Q_update')
        # print(Q_update)

        # modify plan if moveit can't move there: use cartesian path
        # Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints.
        # Configurations are computed for every eef_step meters; The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resultingpath.
        # The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory.
        waypoints = []
        wpose = group.get_current_pose().pose

        for i in range(0,(length_plan2-2)*7,7):
            wpose.position.x = pose_updated[i]
            wpose.position.y = pose_updated[i+1]
            wpose.position.z = pose_updated[i+2]
            wpose.orientation.x = pose_updated[i+3]
            wpose.orientation.y = pose_updated[i+4]
            wpose.orientation.z = pose_updated[i+5]
            wpose.orientation.w = pose_updated[i+6]
            waypoints.append(copy.deepcopy(wpose))
        # append target position
        wpose.position.x = 0.77
        wpose.position.y = -0.58
        wpose.position.z = 0.75-0.78
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]
        waypoints.append(copy.deepcopy(wpose))
        # print('waypoints')
        # print(waypoints)

        # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        # waypoints.append(copy.deepcopy(wpose))
        # wpose.position.y -= scale * 0.1  # Third move sideways (y)
        # waypoints.append(copy.deepcopy(wpose))
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

        # stop_rule_counter = stop_rule_counter + 1
        counter = counter + 1
        print('counter')
        print(counter)

        # print('stop rule norm')
        # print(np.linalg.norm( xyz_updated - xyz_prev))
        # print( np.linalg.norm( (xyz_updated - xyz_prev), keepdims=True).shape )
        # if len(xyz_updated) > len(xyz_prev):
        #     stop_rule_condition = np.linalg.norm( xyz_updated[0:len(xyz_prev)] - xyz_prev)
        #     print(np.linalg.norm( xyz_updated[0:len(xyz_prev)] - xyz_prev))
        # elif len(xyz_updated) < len(xyz_prev):
        #     stop_rule_condition = np.linalg.norm( xyz_updated - xyz_prev[0:len(xyz_updated)])
        #     print(np.linalg.norm( ( xyz_updated - xyz_prev[0:len(xyz_updated)])))
        # else:
        #     stop_rule_condition = np.linalg.norm( xyz_updated - xyz_prev)
        #     print(np.linalg.norm( xyz_updated - xyz_prev))
        # print(stop_rule_counter)
        # print('xyz_updated')
        # print(xyz_updated)
        # print('xyz_prev')
        # print(xyz_prev)
        # print(len(xyz_updated))
        # print(len(xyz_prev))
        length_plan2 = len(plan2.joint_trajectory.points)
        print(length_plan2)
        # print(plan2)
        # print(waypoints)

    robot_state = group.get_current_state()
    # print('robot_state')
    # print(robot_state)
    # plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=1.0, acceleration_scaling_factor=1.0)
    plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

    # The easiest way should be to use the StateValidationService of the move_group node, by calling the service /check_state_validity with this message.
    # You will need the RobotState, which you can get from the /get_planning_scene service with this message.
    # In the definition of the StateValidationService, you also find a pretty good example of how to do the checks yourself, in case you want to avoid calling the service too often.
    # moveit_msgs/GetStateValidity.srv
    # rospy.wait_for_service('/check_state_validity')
    # moveit_state_validity_srv = rospy.ServiceProxy('/check_state_validity', moveit_msgs.srv.GetStateValidity)
    # srv = rospy.ServiceProxy('/check_state_validity',
    #                                moveit_msgs.srv.GetStateValidity)
    # srv.wait_for_service()
    # srv = rospy.ServiceProxy('/check_state_validity',GetStateValidity)
    # print('srv')
    # print(srv)

    # group.execute(plan2, wait=True)

    # print(plan2)
    # print(waypoints)

    index = 0
    # file1 = open("/home/melanie/Desktop/Baxter_poses_potentialf.txt", "w")
    # file1.close()
    # file1 = open("/home/melanie/Desktop/Baxter_poses_potentialf.txt", "a")
    # file1 = open(sys.path.append(os.path.join(absolute_path,'Baxter_poses_potentialf.txt')), "w")
    # file1.close()
    # file1 = open(sys.path.append(os.path.join(absolute_path,'Baxter_poses_potentialf.txt')), "a")
    file1_path = os.path.join(absolute_path,'Baxter_poses_potentialf.txt')
    # print(file1_path)
    file1 = open(file1_path, "w")
    file1.close()
    file1 = open(file1_path, "a")
    print('len(waypoints)')
    print(len(waypoints))
    while index < len(waypoints):
        # print current pose to file
        # writing newline character
        # file1.write("\n")
        file1.write(str(waypoints[index]))
        file1.write("\n")
        index = index + 1
    file1.close()

    # modify plan if moveit can't move there (needed?)
    # how can moveit check if path possible without executing?

    rospy.sleep(5)





if __name__ == '__main__':
    main()
