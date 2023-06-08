import sys
import copy
import math
from math import pi, tau, dist, fabs, cos

import numpy as np

import rospy
import tf2_ros
from std_msgs.msg import String
import geometry_msgs.msg

import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import OrientationConstraint, Constraints

from moveit_commander.conversions import pose_to_list


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_study', anonymous=True)

    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander('arm')

    # moveit_msgs.msg.RobotState
    robot_state = robot.get_current_state()

    # print(robot.get_group_names())
    # print('type(robot_state):', type(robot_state))

    # equivalent to moveit_commander.MoveGroupCommander('panda_arm')?
    # move_group = robot.get_group('panda_arm')

    # print(dir(robot))
    # print(type(move_group), type(move_group.get_current_state()))

    """
    three ways to move a robot
      1. joint
      2. pose
      3. Cartesian path
    """

    # move_group.set_named_target('home')
    # move_group.go(wait=True)

    joint_goal = move_group.get_current_joint_values()
    print(joint_goal)
    t = 1.0
    joint_goal = [0, -t, t * 2, -t - np.pi / 2, -np.pi / 2, 0]

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # # get_current_pose gives only the pose of the end effector?
    ee_pose = move_group.get_current_pose()
    # print('ee pose:', ee_pose)
    # print(ee_pose.pose.position, ee_pose.pose.orientation)

    # ocm = OrientationConstraint()
    #
    # ocm.link_name = 'ee_link'
    # ocm.header.frame_id = 'dummy'
    # ocm.orientation = ee_pose.pose.orientation
    # ocm.absolute_x_axis_tolerance = 0.1
    # ocm.absolute_y_axis_tolerance = 0.1
    # ocm.absolute_z_axis_tolerance = 0.1
    # ocm.weight = 1.0
    #
    # ee_pose.pose.position.y += .1
    #
    # constraints = Constraints()
    # constraints.orientation_constraints.append(ocm)
    # move_group.set_path_constraints(constraints)
    # move_group.set_pose_target(ee_pose)
    # move_group.go(wait=True)
    #
    # ee_pose = move_group.get_current_pose()
    # print('ee pose:', ee_pose)

    # 1. joint value target
    # We get the joint values from the group and change some of the values:
    # for _ in range(2):
    #     joint_goal = move_group.get_current_joint_values()
    #     joint_goal = [v + 0.1 for v in joint_goal]
    #
    #     # The go command can be called with joint values, poses, or without any
    #     # parameters if you have already set the pose or joint target for the group
    #     move_group.go(joint_goal, wait=True)
    #
    #     # Calling ``stop()`` ensures that there is no residual movement
    #     move_group.stop()
    #
    #     joint_goal = move_group.get_current_joint_values()
    #     joint_goal = [v - 0.1 for v in joint_goal]
    #
    #     # The go command can be called with joint values, poses, or without any
    #     # parameters if you have already set the pose or joint target for the group
    #     move_group.go(joint_goal, wait=True)
    #
    #     # Calling ``stop()`` ensures that there is no residual movement
    #     move_group.stop()

    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = np.sqrt(0.5)
    # pose_goal.orientation.x = 0
    # pose_goal.orientation.y = np.sqrt(0.5)
    # pose_goal.orientation.z = 0.
    #
    # pose_goal.position.x = 0.5
    # pose_goal.position.y = 0.5
    # pose_goal.position.z = -0.1
    #
    # move_group.set_pose_target(pose_goal)
    # move_group.go(wait=True)
    #
    # joint_goal = move_group.get_current_joint_values()
    # print(joint_goal)

    # waypoints = []
    #
    # # First move up (z) and sideways (y)
    # w_pose = move_group.get_current_pose().pose
    # w_pose.position.x = 0.25
    # w_pose.position.y += 0.3
    # waypoints.append(copy.deepcopy(w_pose))
    #
    # # Second move forward/backwards in (x)
    # w_pose.position.x = 0.5
    # w_pose.position.y -= 0.3
    # waypoints.append(copy.deepcopy(w_pose))
    #
    # # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # # which is why we will specify 0.01 as the eef_step in Cartesian
    # # translation.  We will disable the jump threshold by setting it to 0.0,
    # # ignoring the check for infeasible jumps in joint space, which is sufficient
    # # for this tutorial.
    # (plan, fraction) = move_group.compute_cartesian_path(
    #     waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    # )  # jump_threshold
    #
    # print(type(plan), type(fraction))
    # move_group.execute(plan, wait=True)


if __name__ == '__main__':
    main()
