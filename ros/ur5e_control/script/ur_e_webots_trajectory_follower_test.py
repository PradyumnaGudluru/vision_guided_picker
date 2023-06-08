#! /usr/bin/env python
import sys

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    # ActionClient only works in a ROS node
    rospy.init_node('ur5_e_controller', sys.argv)

    ac = actionlib.ActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    ac.wait_for_server()

    # FollowJointTrajectoryAction <-> FollowJointTrajectoryGoal
    goal = FollowJointTrajectoryGoal()

    goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                                   'elbow_joint',
                                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    init_position = [0] * 6

    p0 = JointTrajectoryPoint(positions=init_position, velocities=[0] * 6,
                              time_from_start=rospy.Duration(0))
    p1 = JointTrajectoryPoint(positions=[x - 0.5 for x in init_position], velocities=[0] * 6,
                              time_from_start=rospy.Duration(1, 100000))
    p2 = JointTrajectoryPoint(positions=init_position, velocities=[0] * 6,
                              time_from_start=rospy.Duration(2, 200000))

    goal.trajectory.points = [p0, p1, p2]

    def transition_cb(goal_handle):
        print(goal_handle)

    def feedback_cb(goal_handle, feedback):
        print(goal_handle, feedback)

    # you need to keep the reference to the goal handle!
    gh = ac.send_goal(goal, transition_cb, feedback_cb)

    # r = rospy.Rate(10)
    #
    # while not rospy.is_shutdown():
    #     print(gh.get_result())
    #     r.sleep()

    # rospy.spin()
    ac.wait_for_server()
    print(gh.get_goal_status())


if __name__ == '__main__':
    main()
