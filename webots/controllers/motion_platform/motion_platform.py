import numpy as np
from controller import Robot


def main():
    time_step = 32
    robot = Robot()

    outer_motors = [robot.getDevice(f'motor{x}') for x in range(0, 6, 2)]
    inner_motors = [robot.getDevice(f'motor{x}') for x in range(1, 6, 2)]
    
    for motor in outer_motors + inner_motors:
        motor.getPositionSensor().enable(time_step)
    
    outer_pos = [m.getPositionSensor().getValue() for m in outer_motors]
    inner_pos = [m.getPositionSensor().getValue() for m in inner_motors]
    
    th = 0   
    amp = 0.2
    
    while robot.step(time_step) != -1:
        for op, om in zip(outer_pos, outer_motors):
            om.setPosition(op + amp * np.sin(th))
        
        for ip, im in zip(inner_pos, inner_motors):
            im.setPosition(ip - amp * np.sin(th))

        th += 0.1


if __name__ == '__main__':
    main()
