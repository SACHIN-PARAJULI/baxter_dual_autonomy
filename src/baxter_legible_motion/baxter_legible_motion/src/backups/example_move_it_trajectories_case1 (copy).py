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


def load_gazebo_models(table_pose=Pose(position=Point(x=0.7, y=0.0, z=0.0)),
                       table_reference_frame="world",
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

        group.set_joint_value_target(joint_positions)
        # Plan and execute in one command
        group.go(wait=True)
        print('done move to home position')


    # state-of-the-art legible motion
    def trajectory_legible_test(self, startPosition, targetPosition, obstaclePositions):
        # parameters of the solution
        nsteps=15
        pointID = -1
        ptvel = 75
        ptvel2 = 1
        legw = 7.5

        # create Robot
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

    print('calculate state-of-the-art legible trajectory')
    startPosition = (pose_goal.position.x, pose_goal.position.y, pose_goal.position.z)
    solution = example.trajectory_legible_test(startPosition, targetPosition, obstaclePositions)

    x_positions = solution[0::3]
    y_positions = solution[1::3]
    z_positions = solution[2::3]

    waypoints = []
    wpose = group.get_current_pose().pose
    quaternion = [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]
    for i in range(int(len(solution)/3)):
        print(i)
        wpose.position.x = x_positions[i]
        wpose.position.y = y_positions[i]
        wpose.position.z = z_positions[i]
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]
        waypoints.append(copy.deepcopy(wpose))

    (plan2, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.1,        # eef_step
                                       0.0)         # jump_threshold
    # print('plan2')
    # print(plan2)
    print('fraction')
    print(fraction)
    robot_state = group.get_current_state()
    plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

    # uncomment
    group.execute(plan2, wait=True)

    # save waypoints to file to plot trajectory later
    index = 0
    file1_path = os.path.join(absolute_path,'Baxter_poses_legible.txt')
    file1 = open(file1_path, "w")
    file1.close()
    file1 = open(file1_path, "a")
    while index < len(waypoints):
        file1.write(str(waypoints[index]))
        file1.write("\n")
        index = index + 1
    file1.close()



    # calculate potential field

    # uncomment
    example.moveit()
    
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

    iter_points = []
    waypoints_orientation = []

    for i in range(len(plan2.joint_trajectory.points)):
        pose = kdl_kin.forward(np.asarray(plan2.joint_trajectory.points[i].positions))
        iter_points.append( [ np.asarray([ pose.item((0, 3)), pose.item((1, 3)), pose.item((2, 3)) ]), i] )


        quat_updated = quaternion_from_matrix(pose)
        waypoints_orientation.append( quat_updated )


    total_plan = copy.deepcopy(iter_points)

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
    group.execute(plan2, wait=True)

    # save waypoints to file to plot trajectory later
    index = 0
    file1_path = os.path.join(absolute_path,'Baxter_poses_potentialf.txt')
    file1 = open(file1_path, "w")
    file1.close()
    file1 = open(file1_path, "a")
    print('len(waypoints)')
    print(len(waypoints))
    while index < len(waypoints):
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
