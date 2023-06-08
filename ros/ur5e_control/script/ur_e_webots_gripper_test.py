#! /usr/bin/env python
import sys
import rospy

import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommand


def main():
    # ActionClient only works in a ROS node
    rospy.init_node('ur5_e_controller', sys.argv)

    # I used "gripper_controller" in the MoveIt! setup assistant
    # MoveIt! requires exactly "gripper_action" for GripperCommandAction ActionServer!
    ac = actionlib.ActionClient('/gripper_controller/gripper_action', GripperCommandAction)
    ac.wait_for_action_server_to_start()

    goal = GripperCommandGoal()

    # Released 0.05 <-> 0.5 Grasp
    goal.command = GripperCommand(position=0.05, max_effort=-1)

    gh = ac.send_goal(goal)
    ac.wait_for_server()

    print(gh.get_goal_status())


if __name__ == '__main__':
    main()
