from math import fabs

import actionlib
from control_msgs.msg import GripperCommandAction

from actionlib_msgs.msg import GoalStatus


class GripperCommander:
    joint_names = ['finger_1_joint_1', 'finger_2_joint_1', 'finger_middle_joint_1']

    def __init__(self, robot, node_name):
        self.robot = robot
        self.time_step = int(robot.getBasicTimeStep())

        self.goal_handle = None
        self.position = None
        self.max_effort = None

        self.motors = [robot.getDevice(n) for n in GripperCommander.joint_names]
        self.sensors = [robot.getDevice(n + '_sensor') for n in GripperCommander.joint_names]
        for sensor in self.sensors:
            sensor.enable(self.time_step)

        # note that MoveIt! requires exactly "gripper_action" here
        # or the default action_ns is "gripper_action"
        self.server = actionlib.ActionServer(node_name + "gripper_action",
                                             GripperCommandAction,
                                             self.on_goal, self.on_cancel, auto_start=False)

    def start(self):
        self.server.start()

    def on_goal(self, goal_handle):
        self.goal_handle = goal_handle
        goal_handle.set_accepted()

        goal = goal_handle.get_goal()
        self.position = goal.command.position
        self.max_effort = goal.command.max_effort

        for motor in self.motors:
            motor.setPosition(self.position)

        dir(goal_handle)

    def on_cancel(self, goal_handle):
        if self.goal_handle == goal_handle:
            for motor, sensor in zip(self.motors, self.sensors):
                motor.setPosition(sensor.getValue())
            self.goal_handle = None

        goal_handle.set_cancled()

    def update(self):
        gh = self.goal_handle

        # I think it might be useful to keep the last successful goal
        if gh and gh.get_goal_status().status == GoalStatus.ACTIVE:
            if all([fabs(sensor.getValue() - self.position) < 0.01 for sensor in self.sensors]):
                self.goal_handle.set_succeeded()
