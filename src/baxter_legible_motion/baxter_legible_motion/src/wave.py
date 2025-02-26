#!/usr/bin/python3

import rospy
import moveit_commander
import sys
import moveit_msgs.msg
import geometry_msgs.msg

if __name__ == '__main__':
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander('right_arm')

    joint_goal_left = group.get_current_joint_values()
    # print(joint_goal_left)

    joint_goal_left = [0.34898062924393164, 0.7551020428365949, 2.8673935877548096, 1.445393397385031, -3.0541557486798587, -1.0528788085054117, 0.769291365124535]

    joint_goal_right = group.get_current_joint_values()
    # print(joint_goal_right)

    joint_goal_right = [0.34898062924393164, 0.7551020428365949, 2.3673935877548096, 1.445393397385031, -3.0541557486798587, -1.0528788085054117, 0.769291365124535]

    group.set_max_velocity_scaling_factor(0.7)

    # sys.exit(0)

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group\
    while not rospy.is_shutdown():
        group.go(joint_goal_left, wait=True)
        rospy.sleep(.5)

        group.go(joint_goal_right, wait=True)
        rospy.sleep(.5)
