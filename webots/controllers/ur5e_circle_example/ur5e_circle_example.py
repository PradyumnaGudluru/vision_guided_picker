import os

import controller
import numpy as np

from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics import Frame


robot = controller.Robot()

time_step = 32

joint_names = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

motors = [robot.getDevice(motor) for motor in joint_names]

pi = np.pi
dh_params = np.asarray([
    [0.1625, 0, 0.5 * pi, 0],
    [0, -0.425, 0, 0],
    [0, -0.3922, 0, 0],
    [0.1333, 0., 0.5 * pi, 0],
    [0.0997, 0., -0.5 * pi, 0],
    [0.0996, 0, 0, 0]])
vk_robot = RobotSerial(dh_params)

th = 0

while robot.step(time_step) != -1:
    xyz = np.array([[-0.6 + 0.2 * np.cos(th)], [0.2 * np.sin(th)], [0.25]])
    abc = np.array([np.pi, 0, 0])
    end = Frame.from_r_3(abc, xyz)
    vk_robot.inverse(end)
    
    for angle, motor in zip(vk_robot.axis_values, motors):
        motor.setPosition(angle)

    th += 0.05
