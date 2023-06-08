import glob
import os
import sys
import random

import numpy as np
from scipy.spatial.transform import Rotation as R
from PIL import Image

from controller import Robot


def main():
    time_step = 32
    
    pix_names = glob.glob('/home/young/Pictures/orange_field_views/*.png')
    pix_nums = [int(os.path.basename(x).split('.')[0]) for x in pix_names]
    cur_pix_num = max(pix_nums) + 1 if pix_nums else 0
    pix_count = 0
    
    robot = Robot()
    
    x_motor = robot.getDevice('x motor')
    y_motor = robot.getDevice('y motor')
    camera_motor = robot.getDevice('rotational motor')
    
    camera = robot.getDevice('camera')
    camera.enable(time_step)
    
    range_finder = robot.getDevice('range-finder')
    range_finder.enable(time_step)
    
    
    dir = 1
    y_dir = 1
    
    x, y = 0, 0
    th = 0
    
    while robot.step(time_step) != -1:
        x += dir * 0.01
        th += 0.1
        
        y += y_dir * 0.005
        
        if x > 4:
            dir = -1
        elif x < -4:
            dir = 1
            
        if y > 0.275:
            y_dir = -1
        elif y < -0.275:
            y_dir = 1
        
        x_motor.setPosition(x)
        y_motor.setPosition(y)
        camera_motor.setPosition(th)
        
        if pix_count < 20 and random.random() < 0.01:
            pix = np.swapaxes(np.asarray(camera.getImageArray(), dtype=np.uint8), 0, 1)
            img = Image.fromarray(pix)
            img.save("/home/young/Pictures/orange_field_views/{0:d}.png".format(cur_pix_num))
            cur_pix_num += 1
            pix_count += 1
            
            if pix_count > 19:
                print('20 pictures taken.')


if __name__ == '__main__':
    main()
