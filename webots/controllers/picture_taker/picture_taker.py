from controller import Supervisor
import sys

import numpy as np
from PIL import Image


time_step = 32

supervisor = Supervisor()

# do this once only
# robot_node = supervisor.getFromDef("robot")
robot_node = supervisor.getSelf()

if robot_node is None:
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)

camera = supervisor.getDevice("camera")
camera.enable(time_step)

range_finder = supervisor.getDevice("range-finder")
range_finder.enable(time_step)

# camera_node = robot_node.getDevice('camera')
# print(camera_node)

# (6.2, -0.75) -> (4.7, 0.25) -> (3.2, -0.75)
direction = 0
th = 0

trans_field = robot_node.getField("translation")
rot_field = robot_node.getField("rotation")

i = -25
j = 0
picture_taken_at = set()

while supervisor.step(time_step) != -1:
    lower_bound = -np.pi / 6
    upper_bound = np.pi + np.pi / 6

    th = i * np.pi / 120
    x = 1.5 * np.cos(th) + 4.7
    y = np.sin(th) - 0.75

    trans_field.setSFVec3f([x, y, 0.93])
    rot_field.setSFRotation([0, 0, 1, th + np.pi])

    if i % 10 == 0:
        if i not in picture_taken_at:
            pix = np.swapaxes(np.asarray(camera.getImageArray(), dtype=np.uint8), 0, 1)
            img = Image.fromarray(pix)
            img.save("/home/young/Pictures/img_{0:02d}.jpg".format(j))
            j += 1
            picture_taken_at.add(i)

    if direction == 0:
        i += 1
        if th > upper_bound:
            direction = 1
    else:
        i -= 1
        if th < lower_bound:
            direction = 0
