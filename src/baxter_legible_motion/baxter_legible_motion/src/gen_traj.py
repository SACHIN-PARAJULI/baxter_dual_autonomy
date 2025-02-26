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

def get_random_points(num_points, max_x, min_x, max_y, min_y):
    to_ret = []

    for i in range(num_points):
        x = random.random() * (max_x - min_x) + min_x
        y = random.random() * (max_y - min_y) + min_y

        to_ret.append(np.array([x, y]))

    return np.transpose(np.atleast_2d(to_ret))

def KL_divergence_gaussian_uniform (cov, q):
    if np.linalg.det(cov) <= 0:
        return 0

    return -1 - math.log(2 * math.pi) - math.log( np.linalg.det(cov) ) - math.log( q )

# This is the maximum reasonable KL_divergence (otherwise objects would have to be stacked on top of one another)
def max_KL_divergence (max_x, min_x, max_y, min_y, q_const):
    dumb_points = get_random_points(20, max_x, min_x, max_y, min_y)

    point1 = [ dumb_points[0][0], dumb_points[0][1] ]

    dumb_points = np.transpose(np.atleast_2d([ [ point1[0] + .25 * random.random(), point1[1] + .25 * random.random() ] for i in range(len(dumb_points[0]))]))

    cov_mat = np.cov(dumb_points)

    return KL_divergence_gaussian_uniform(cov_mat, q_const)

# xsi is the KL_divergence of the estimated gaussian which represents the points in objects compared to a uniform distribution in the same area
def calculate_xi( objects, mins, maxs ):

    q_const = 1

    if ( maxs[0] - mins[0] ) == 0 and ( maxs[1] - mins[1] ) == 0:
        q_const = 1
    elif ( maxs[0] - mins[0] ) == 0:
        q_const =  1 /  ( maxs[1] - mins[1] )
    elif ( maxs[1] - mins[1] ) == 0:
        q_const =  1 / ( maxs[0] - mins[0] )
    else:
        q_const =  1 / ( ( maxs[0] - mins[0] ) * ( maxs[1] - mins[1] ) )

    objects_temp = np.transpose(np.atleast_2d(objects))

    max_kl = max_KL_divergence(maxs[0], mins[0], maxs[1], mins[1], q_const)

    cov_mat = np.cov(objects_temp[:2])
    kl_score = KL_divergence_gaussian_uniform(cov_mat, q_const)

    print(f'{kl_score} / {max_kl} = {kl_score / max_kl}')

    return kl_score / max_kl


def calc_psi(startPosition, targetPosition, objectPosition):
    cpa = ( np.linalg.norm( np.cross( (np.array([objectPosition[0], objectPosition[1]]) - np.array([startPosition[0], startPosition[1]])) , \
            (np.array([ targetPosition[0], targetPosition[1] ]) - np.array([startPosition[0], startPosition[1]])) ) ) ) \
            / np.linalg.norm( (np.array([ targetPosition[0], targetPosition[1] ]) - np.array([startPosition[0], startPosition[1]])) )

    psi = math.asin( cpa / math.dist([startPosition[0], startPosition[1]], [ objectPosition[0], objectPosition[1] ]))

    psi /= ( math.pi / 2 )

    return 1 - psi


def load_gazebo_models(objects):

    table_pose=Pose(position=Point(x=0.7, y=0.0, z=0.0))
    table_reference_frame="world"

    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_gazebo')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')

    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


    for i in range(len(objects)):

        mug_pose = Pose(position=Point(x=objects[i][0], y=objects[i][1], z=0.75-objects[i][2]))
        mug_reference_frame = "world"

        # Load mug URDF
        mug_xml = ''
        with open (model_path + "025_mug/mug.sdf", "r") as mug_file:
            mug_xml=mug_file.read().replace('\n', '')
        # Spawn Table SDF
        rospy.wait_for_service('/gazebo/spawn_sdf_model')

        # Spawn mug URDF
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("mug" + str(i), mug_xml, "/",
                                 mug_pose, mug_reference_frame)
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
        for i in range(len(objects)):
            resp_delete = delete_model("mug" + str(i))
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

        self.marker[index].scale.x = 0.05
        self.marker[index].scale.y = 0.05
        self.marker[index].scale.z = 0.05

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
        for index in range(len(objects)):
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

def load_points_from_param_server():
    loaded_objects = []

    point_names = rospy.get_param('/point_names')

    for name in point_names:
        x = rospy.get_param('/points/' + name + '/x')
        y = rospy.get_param('/points/' + name + '/y')
        z = rospy.get_param('/points/' + name + '/z')

        loaded_objects.append(np.array([x, y, z]))

    return loaded_objects

def get_waypoints_from_path(move_group, cartesian_path):
    waypoints = []
    x_positions = cartesian_path[0::3]
    y_positions = cartesian_path[1::3]
    z_positions = cartesian_path[2::3]

    wpose = move_group.get_current_pose().pose
    quaternion = [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]
    for i in range(int(len(cartesian_path)/3)):
        print(i)
        wpose.position.x = x_positions[i]
        wpose.position.y = y_positions[i]
        wpose.position.z = z_positions[i]
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]
        waypoints.append(copy.deepcopy(wpose))

    return waypoints

def generate_pf_plan(startPosition, targetPosition, obstaclePositions, xi, oStartPosition):
    # parameter for deciding what close enough to the goal is (1cm)
    epsilon = 0.01

    # gains parameters
    k_rep = .03
    k_rep_2d = .025 * ( 1 - xi )
    k_attr = 0.45 #attr = attractive

    # final gain param (not sure if necessary/could be that this is covered by xsi)
    k_update = 0.2

    # params for the size of each respective potential field
    ## we don't want to immediately push the hand up so the max height is slightly less than the height of the start position
    max_height = math.dist([ startPosition[0], startPosition[1], targetPosition[2]], startPosition) - .005
    ## the max 2d range is scaled by xsi so that more cluttered environments have less max range
    ### NOTE: this works for now but I think that I should replace this with max_range * ( 1 - xsi )
    max_range_2d = .3 *  (1 - xi)
    ## this is the ellipsoidal limit for the z direction
    ro_max = 2 * max_height

    # the max number of point is 201 and we assume to not have converged
    max_num = 300

    # output of the algorithm is stored in total_plan
    total_plan = [ list(startPosition) ]

    # either we come within range of the goal or get stuck to stop
    while np.linalg.norm( targetPosition - total_plan[-1] ) > epsilon and len(total_plan) < max_num:

        U_grad_rep_total = np.array([0.0, 0.0, 0.0])

        eePosition = np.array(total_plan[-1])

        for j in obstaclePositions:

            ##### get repulsive force for each object in the obstacles #####
            ### force in z direction calculated using the equation of an ellipsoid ###

            # we want the radius to end at the target so that the hand can always fall towards it
            radius = math.dist(j, targetPosition)

            # enforce the max x,y range
            if radius > max_height:
                radius = max_height

            ## calculation for the placement of the two focal points of the ellipsoid ##
            a = max_height * math.cos( math.asin( radius / max_height ) )

            f1 = np.array([ j[0], j[1], j[2] - a ])
            f2 = np.array([ j[0], j[1], j[2] + a ])

            ## now the force calculation ##
            # current distance from ellipsoid focal points
            ro_b = math.dist(eePosition, f1) + math.dist(eePosition, f2)

            # max distance of ellipsoid
            ro_0 = math.dist(f1, targetPosition) + math.dist(f2, targetPosition)

            # enforce the max x,y range
            if ro_0 > ro_max:
                ro_0 = ro_max

            ## now the actual force calculation ##
            force_sphere = None

            if ro_b <= ro_0 and eePosition[2] < total_plan[0][2]:
                force_sphere = k_rep * (1/ro_0 - 1/ro_b ) * ((2 * eePosition - f1 - f2)/ro_b)
            else:
                force_sphere = [ 0, 0, 0 ]

            ### x,y force calculation using the equation of a circle with no regard for height ###
            ro_b = math.dist([eePosition[0], eePosition[1]], [ j[0], j[1] ] )

            ro_0 = math.dist([j[0], j[1]], [ targetPosition[0], targetPosition[1] ] )

            psi = calc_psi(oStartPosition, targetPosition, j)

            ## 0 out psi both if the startPosition is closer to the target than the object, or if the distance from the
            #### object is greater than the distance from the startPosition to the targetPosition
            # if ro_b >= math.dist([eePosition[0], eePosition[1]], [ targetPosition[0], targetPosition[1] ]) or \
            #     ro_0 >= math.dist([oStartPosition[0], oStartPosition[1]], [ targetPosition[0], targetPosition[1] ]) :
            #     psi = 0

            # scale the max_range by psi
            # if ro_0 > max_range_2d:
            #     ro_0 = max_range_2d

            ## calculate the 2d force, this time without taking height into account ##
            force_2d = None

            if ro_b <= ro_0:
                force_2d = k_rep_2d * (1/ro_0 - 1/ro_b ) * ( ( np.array([eePosition[0], eePosition[1]]) - np.array([j[0], j[1]]) ) / ro_b )
            else:
                force_2d = np.array([ 0, 0 ])

            # scale the force by psi
            force_2d = force_2d * psi

            # negative z goes through the tabe
            # z_force = force_sphere[2]
            #
            # if z_force < 0:
            #     z_force *= -1

            ### combine the two forces ###
            U_grad_rep = [force_2d[0], force_2d[1], force_sphere[2]]

            ##### add them to the total repulsive force
            U_grad_rep_total += U_grad_rep

        ### calculate the attractive force and the total force for the update rule ###
        ## scale the total repulsive force by xsi
        U_grad_rep_total = U_grad_rep_total * ( 1 - xi )

        ## the attractive force is linear in the direction of the target
        U_grad_attr = k_attr * (eePosition - targetPosition)

        ### this -1 is taken out of earlier calculations and finally added back here
        force = -1 * (U_grad_rep_total + U_grad_attr)

        # add the new point to the plan
        new_point = total_plan[-1] + k_update * force
        total_plan.append(new_point)

    return total_plan

def clean_trajectory(waypoints):
    to_remove = [ 1 ]

    while len(to_remove) > 0:
        to_remove = []

        # min total_distance between 3 points is 3 cm
        epsilon = .03

        i = 1

        # mark for removal all the points which are too close to each other
        while i < len(waypoints) - 1:
            if math.dist(waypoints[i - 1], waypoints[i]) + math.dist(waypoints[i], waypoints[i + 1]) < epsilon:

                to_remove.append(i)
                i += 2
            else:
                i += 1

        to_remove.reverse()

        # now remove them in reverse order so that none of the indices shift
        for i in to_remove:

            del waypoints[i]

def get_smooth_legible_trajectory(startPosition, targetPosition, obstaclePositions, xi):
    total_waypoints = []

    # get pf trajectory
    waypoints = generate_pf_plan(startPosition, targetPosition, obstaclePositions, xi, startPosition)

    # find local maximums
    local_maxs = [ startPosition ]

    for i in range(1, len(waypoints) - 1):
        if waypoints[i - 1][2] <= waypoints[i][2] and waypoints[i + 1][2] <= waypoints[i][2]:
            local_maxs.append( copy.deepcopy( waypoints[i] ) )

    local_maxs.append(targetPosition)

    # plan from main point to local max
    for i in range(len(local_maxs) - 1):
        n_waypoints = generate_pf_plan(local_maxs[i], local_maxs[i + 1], obstaclePositions, xi, startPosition)

        for j in n_waypoints:
            total_waypoints.append(copy.deepcopy(j))

    clean_trajectory(total_waypoints)

    # return updated total trajectory
    return total_waypoints

def run_study(seed):

    random.seed(seed)

    example = ExampleMoveItTrajectories()

    ##### initial setup for the whole algo #####
    # load object positions for the experiment
    objects = load_points_from_param_server()

    # get target and obstacle positions
    group = example.group

    group.set_max_velocity_scaling_factor(0.7)

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -1.300942808659576
    joint_goal[1] = -1.1064183104668466
    joint_goal[2] = 1.1765210844176874
    joint_goal[3] = 2.1887659552021455
    joint_goal[4] = -0.5662488954816365
    joint_goal[5] =  0.8754244383386937
    joint_goal[6] = -2.508083413019744

    max_x = -1000
    max_y = -1000
    min_x = 1000
    min_y = 1000

    # load_gazebo_models(objects)
    for index in range(int(len(objects))):
        example.marker_add(index, objects[index])

    # Remove models from the scene on shutdown
    # rospy.on_shutdown(delete_gazebo_models)
    rospy.on_shutdown(example.marker_delete)

    experiment = rospy.get_param('/full_study')

    if len(experiment) < 10:
        random.shuffle(experiment)
    else:
        tmp = experiment[:10]

        random.shuffle(tmp)

        experiment = tmp + experiment[10:]

    group.go(joint_goal, wait=True)

    input()

    print(experiment)

    for i in experiment:
        obstaclePositions = []
        targetPosition = None

        ### get object number and which planner to use from list in experiment yaml
        params = i.split('_')

        object_num = int(params[0])
        sota = params[1] == 's'

        print(f'### planning to grab object {object_num} with the ', end='')
        if sota:
            print('state of the art planner!')
        else:
            print('potential fields planner!')

        group.go(joint_goal, wait=True)

        input()

        for i in range(len(objects)):
            # find max and min for x and y
            if max_x < objects[i][0]:
                max_x = objects[i][0]
            if max_y < objects[i][1]:
                max_y = objects[i][1]
            if min_x > objects[i][0]:
                min_x = objects[i][0]
            if min_y > objects[i][1]:
                min_y = objects[i][1]

            # also pick the target object out of the lineup and put the rest in the obstacles list
            if i == object_num:
                targetPosition = copy.deepcopy(objects[i])
            else:
                obstaclePositions.append(copy.deepcopy(objects[i]))

        pose_goal = group.get_current_pose().pose
        startPosition = (pose_goal.position.x, pose_goal.position.y, pose_goal.position.z)
        solution = example.trajectory_legible_test(startPosition, targetPosition, obstaclePositions)

        if sota:
            ### calculate state of the art trajectory for comparison ###
            # example.moveit()

            # print('calculate state-of-the-art legible trajectory')

            # targetPosition[2] -= .02

            waypoints_sota = get_waypoints_from_path(group, solution)

            (plan2, fraction) = group.compute_cartesian_path(
                                               waypoints_sota[:math.floor(len(waypoints_sota) / 3)],   # waypoints to follow
                                               0.1,         # eef_step
                                               0.0)         # jump_threshold

            robot_state = group.get_current_state()
            plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

            # uncomment to send an execution goal to the robot
            group.execute(plan2, wait=True)
            input()
            # split trajectory into 3 parts

            # get a moveit plan through the generated points
            (plan2, fraction) = group.compute_cartesian_path(
                                               waypoints_sota[math.floor(len(waypoints_sota) / 3):math.floor(len(waypoints_sota) * 2 / 3)],   # waypoints to follow
                                               0.1,         # eef_step
                                               0.0)         # jump_threshold

            robot_state = group.get_current_state()
            plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

            group.execute(plan2, wait=True)
            input()
            # split trajectory into 3 parts

            # get a moveit plan through the generated points
            (plan2, fraction) = group.compute_cartesian_path(
                                               waypoints_sota[math.floor(len(waypoints_sota) * 2 / 3):],   # waypoints to follow
                                               0.1,         # eef_step
                                               0.0)         # jump_threshold

            robot_state = group.get_current_state()
            plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

            group.execute(plan2, wait=True)

        ### calculate potential field ###
        # at the top of this general parameters are set and then the potential field is used to get a path
        # example.moveit()


        else:
            # xsi is calculated once after the points are set up
            xi = calculate_xi(objects, [ min_x, min_y ], [ max_x, max_y ])

            # generate pf plan uses the pf eqs to generate legible motion
            # total_plan = generate_pf_plan(startPosition, targetPosition, obstaclePositions, xi)
            total_plan = get_smooth_legible_trajectory(startPosition, targetPosition, obstaclePositions, xi)

            # make plan look like a ros message
            waypoints_pf = []
            wpose = group.get_current_pose().pose

            for i in range(len(total_plan)):
                wpose.position.x = total_plan[i][0]
                wpose.position.y = total_plan[i][1]
                wpose.position.z = total_plan[i][2]
                wpose.orientation.x = pose_goal.orientation.x
                wpose.orientation.y = pose_goal.orientation.y
                wpose.orientation.z = pose_goal.orientation.z
                wpose.orientation.w = pose_goal.orientation.w
                waypoints_pf.append(copy.deepcopy(wpose))

            # get a moveit plan through the generated points
            (plan2, fraction) = group.compute_cartesian_path(
                                               waypoints_pf[:math.floor(len(waypoints_pf) / 3)],   # waypoints to follow
                                               0.1,         # eef_step
                                               0.0)         # jump_threshold

            robot_state = group.get_current_state()
            plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

            # uncomment to send an execution goal to the robot
            group.execute(plan2, wait=True)
            input()
            # split trajectory into 3 parts

            # get a moveit plan through the generated points
            (plan2, fraction) = group.compute_cartesian_path(
                                               waypoints_pf[math.floor(len(waypoints_pf) / 3):math.floor(len(waypoints_pf) * 2 / 3)],   # waypoints to follow
                                               0.1,         # eef_step
                                               0.0)         # jump_threshold

            robot_state = group.get_current_state()
            plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

            group.execute(plan2, wait=True)
            input()

            (plan2, fraction) = group.compute_cartesian_path(
                                               waypoints_pf[math.floor(len(waypoints_pf) * 2 / 3):],   # waypoints to follow
                                               0.1,         # eef_step
                                               0.0)         # jump_threshold

            robot_state = group.get_current_state()
            plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

            group.execute(plan2, wait=True)

        input()


def main(execute, graph):
    example = ExampleMoveItTrajectories()

    ##### initial setup for the whole algo #####
    # load object positions for the experiment
    objects = load_points_from_param_server()

    # get target and obstacle positions
    group = example.group

    group.set_max_velocity_scaling_factor(0.7)

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -1.300942808659576
    joint_goal[1] = -1.1064183104668466
    joint_goal[2] = 1.1765210844176874
    joint_goal[3] = 2.1887659552021455
    joint_goal[4] = -0.5662488954816365
    joint_goal[5] =  0.8754244383386937
    joint_goal[6] = -2.508083413019744

    max_x = -1000
    max_y = -1000
    min_x = 1000
    min_y = 1000

    # load_gazebo_models(objects)
    for index in range(int(len(objects))):
        example.marker_add(index, objects[index])

    # Remove models from the scene on shutdown
    # rospy.on_shutdown(delete_gazebo_models)
    rospy.on_shutdown(example.marker_delete)

    while True:
        obstaclePositions = []
        targetPosition = None

        ### get object number from user
        print(f'Please enter the number of the object to grab (-1 to stop, {len(objects)} for home position): ', end='')

        object_num = input()

        try:
            object_num = int(object_num)
            if object_num > len(objects):
                raise Exception()
        except:
            print(f'Please enter an integer between 0 and {len(objects)}!')
            continue

        if object_num < 0:
            print('Exiting program...')
            break

        sota = False

        if execute:
            print('Please enter which planner you would like to execute (\'s\' for state of the art, \'p\' for potential fields): ')
            sota_char = input()

            sota = sota_char == 's'

            group.go(joint_goal, wait=True)

        if object_num == len(objects):
            continue

        input()

        for i in range(len(objects)):
            # find max and min for x and y
            if max_x < objects[i][0]:
                max_x = objects[i][0]
            if max_y < objects[i][1]:
                max_y = objects[i][1]
            if min_x > objects[i][0]:
                min_x = objects[i][0]
            if min_y > objects[i][1]:
                min_y = objects[i][1]

            # also pick the target object out of the lineup and put the rest in the obstacles list
            if i == object_num:
                targetPosition = copy.deepcopy(objects[i])
            else:
                obstaclePositions.append(copy.deepcopy(objects[i]))

        ### calculate state of the art trajectory for comparison ###
        # example.moveit()

        # print('calculate state-of-the-art legible trajectory')
        pose_goal = group.get_current_pose().pose
        startPosition = (pose_goal.position.x, pose_goal.position.y, pose_goal.position.z)
        solution = example.trajectory_legible_test(startPosition, targetPosition, obstaclePositions)

        # targetPosition[2] -= .02

        waypoints_sota = get_waypoints_from_path(group, solution)

        if execute and sota:
            (plan2, fraction) = group.compute_cartesian_path(
                                               waypoints_sota[:math.floor(len(waypoints_sota) / 3)],   # waypoints to follow
                                               0.1,         # eef_step
                                               0.0)         # jump_threshold

            robot_state = group.get_current_state()
            plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

            # uncomment to send an execution goal to the robot
            group.execute(plan2, wait=True)
            input()
            # split trajectory into 3 parts

            # get a moveit plan through the generated points
            (plan2, fraction) = group.compute_cartesian_path(
                                               waypoints_sota[math.floor(len(waypoints_sota) / 3):math.floor(len(waypoints_sota) * 2 / 3)],   # waypoints to follow
                                               0.1,         # eef_step
                                               0.0)         # jump_threshold

            robot_state = group.get_current_state()
            plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

            group.execute(plan2, wait=True)
            input()
            # split trajectory into 3 parts

            # get a moveit plan through the generated points
            (plan2, fraction) = group.compute_cartesian_path(
                                               waypoints_sota[math.floor(len(waypoints_sota) * 2 / 3):],   # waypoints to follow
                                               0.1,         # eef_step
                                               0.0)         # jump_threshold

            robot_state = group.get_current_state()
            plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

            group.execute(plan2, wait=True)

        ### calculate potential field ###
        # at the top of this general parameters are set and then the potential field is used to get a path
        # example.moveit()

        # xsi is calculated once after the points are set up
        xi = calculate_xi(objects, [ min_x, min_y ], [ max_x, max_y ])

        # generate pf plan uses the pf eqs to generate legible motion
        # total_plan = generate_pf_plan(startPosition, targetPosition, obstaclePositions, xi)
        total_plan = get_smooth_legible_trajectory(startPosition, targetPosition, obstaclePositions, xi)

        # make plan look like a ros message
        waypoints_pf = []
        wpose = group.get_current_pose().pose

        for i in range(len(total_plan)):
            wpose.position.x = total_plan[i][0]
            wpose.position.y = total_plan[i][1]
            wpose.position.z = total_plan[i][2]
            wpose.orientation.x = pose_goal.orientation.x
            wpose.orientation.y = pose_goal.orientation.y
            wpose.orientation.z = pose_goal.orientation.z
            wpose.orientation.w = pose_goal.orientation.w
            waypoints_pf.append(copy.deepcopy(wpose))

        if execute and not sota:
            # get a moveit plan through the generated points
            (plan2, fraction) = group.compute_cartesian_path(
                                               waypoints_pf[:math.floor(len(waypoints_pf) / 3)],   # waypoints to follow
                                               0.1,         # eef_step
                                               0.0)         # jump_threshold

            robot_state = group.get_current_state()
            plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

            # uncomment to send an execution goal to the robot
            group.execute(plan2, wait=True)
            input()
            # split trajectory into 3 parts

            # get a moveit plan through the generated points
            (plan2, fraction) = group.compute_cartesian_path(
                                               waypoints_pf[math.floor(len(waypoints_pf) / 3):math.floor(len(waypoints_pf) * 2 / 3)],   # waypoints to follow
                                               0.1,         # eef_step
                                               0.0)         # jump_threshold

            robot_state = group.get_current_state()
            plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

            group.execute(plan2, wait=True)
            input()

            (plan2, fraction) = group.compute_cartesian_path(
                                               waypoints_pf[math.floor(len(waypoints_pf) * 2 / 3):],   # waypoints to follow
                                               0.1,         # eef_step
                                               0.0)         # jump_threshold

            robot_state = group.get_current_state()
            plan2 = group.retime_trajectory(robot_state, plan2, velocity_scaling_factor=0.5, acceleration_scaling_factor=0.5)

            group.execute(plan2, wait=True)

        if graph:

            sota_x = []
            sota_y = []
            sota_z = []

            for i in waypoints_sota:
                sota_x.append(i.position.x)
                sota_y.append(i.position.y)
                sota_z.append(i.position.z)

            pf_x = []
            pf_y = []
            pf_z = []

            for i in waypoints_pf:
                pf_x.append(i.position.x)
                pf_y.append(i.position.y)
                pf_z.append(i.position.z)

            ax = plt.figure().add_subplot(projection='3d')

            # state of the art plots
            redDots = plt.plot(sota_x, sota_y, sota_z, c='b', marker='o')[0] # For scatter plot
            line = plt.plot(sota_x, sota_y, sota_z, c='b', label ="State-of-the-Art")[0] # For line plot
            # potential fields plots
            redDots = plt.plot(pf_x, pf_y, pf_z, c='c', marker='o')[0] # For scatter plot
            line = plt.plot(pf_x, pf_y, pf_z, c='c', label ="Ours")[0] # For line plot

            # AXES PROPERTIES
            ax.set_xlabel('X(t)')
            ax.set_ylabel('Y(t)')
            ax.set_zlabel('Z(t)')
            ax.set_title('Trajectory')

            for i in objects:
                ax.scatter3D(i[0], i[1], i[2], s=50)

            ax.view_init(elev=20., azim=-35, roll=0)

            # Function add a legend
            plt.legend(bbox_to_anchor =(0.75, 0.0), ncol = 2)

            plt.show()

        else:
            # save waypoints to file to plot trajectory later for state of the art
            index = 0
            file1_path = os.path.join(absolute_path,'Baxter_poses_legible.txt')

            with open(file1_path, "w") as file1:
                while index < len(waypoints_sota):
                    file1.write(str(waypoints_sota[index]))
                    file1.write("\n")
                    index = index + 1

            # save waypoints to file to plot trajectory later for potential fields
            index = 0
            file1_path = os.path.join(absolute_path,'Baxter_poses_potentialf.txt')

            with open(file1_path, 'w+') as file1:
                print(f'len(waypoints): {len(waypoints_pf)}')
                while index < len(waypoints_pf):
                    file1.write(str(waypoints_pf[index]))
                    file1.write("\n")
                    index = index + 1

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        prog='ProgramName',
        description='What the program does',
        epilog='Text at the bottom of help')

    parser.add_argument('-g', '--graph', action='store_true')
    parser.add_argument('-x', '--execute', action='store_true')
    parser.add_argument('--run_study', action='store_true')
    parser.add_argument('-s', '--seed', default='participant_0')

    args = []
    skip = False

    for i in range(len(sys.argv)):
        if skip:
            skip = False
            continue

        if sys.argv[i] == '-g' or sys.argv[i] == '--graph' or sys.argv[i] == '-x' or sys.argv[i] == '--execute' or sys.argv[i] == '--run_study':
            args.append(sys.argv[i])
        elif '-s' == sys.argv[i] or '--seed' == sys.argv[i]:
            args.append(sys.argv[i])
            args.append(sys.argv[i + 1])
            skip = True

    args = parser.parse_args(args)

    if args.run_study:
        run_study(args.seed)
    else:
        main(args.execute, args.graph)
