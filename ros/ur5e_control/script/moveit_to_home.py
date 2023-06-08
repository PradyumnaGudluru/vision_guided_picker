import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_study', anonymous=True)

    move_group = moveit_commander.MoveGroupCommander('arm')

    print(move_group.get_named_targets())
    joint_names = move_group.get_active_joints()
    home_values = move_group.get_named_target_values('home')

    move_group.set_named_target('home')
    move_group.go(wait=True)
    # Calling ``stop()`` ensures that there is no residual movement
    # move_group.stop()

    goal = [home_values[n] + 0.2 for n in joint_names]
    move_group.go(goal, wait=True)

    move_group.set_named_target('reset')
    move_group.go(wait=True)
    move_group.stop()


if __name__ == "__main__":
    main()
