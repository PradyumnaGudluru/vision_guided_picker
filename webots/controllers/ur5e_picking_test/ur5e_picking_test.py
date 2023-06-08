from controller import Robot


def main():
    time_step = 32
    robot = Robot()

    # ROBOTIQ 3f Gripper 11 rotary motors
    # finger_1_joint_[123] -> 3 motors
    # finger_2_joint_[123] -> 3 motors
    # finger_middle_joint_[123] -> 3 motors
    # palm_finger_[12]_joint -> 2 motors

    # but, here we only use 3 motors
    finger_joint_names = [
        'finger_1_joint_1',
        'finger_2_joint_1',
        'finger_middle_joint_1'
    ]

    # Robotiq3fGripper.proto is attached to toolSlot of UR5e.proto
    # this robot controller for UR5e also has access to Robotiq3fGripper motors!
    finger_joints = [robot.getDevice(name) for name in finger_joint_names]
    finger_sensors = []

    for name in finger_joint_names:
        finger_sensor = robot.getDevice(name + '_sensor')
        finger_sensor.enable(time_step)
        finger_sensors.append(finger_sensor)

    # 0.85
    # get_min_position
    shoulder_pan_joint = robot.getDevice('shoulder_pan_joint')

    shoulder_lift_sensor = robot.getDevice('shoulder_pan_joint_sensor')
    shoulder_lift_sensor.enable(time_step)

    state = 0

    while robot.step(time_step) != -1:
        if state == 0:
            # start to back off
            shoulder_pan_joint.setPosition(-0.5)
            state = 1
        elif state == 1:
            # backing off
            if abs(shoulder_lift_sensor.getValue() - (-0.5)) < 0.01:
                shoulder_pan_joint.setPosition(0.)
                state = 2
        elif state == 2:
            # approaching to the target
            if abs(shoulder_lift_sensor.getValue() - 0) < 0.01:
                # it reached to the target!
                # start to grasp it!
                for finger in finger_joints:
                    finger.setPosition(0.5)
                state = 3
        elif state == 3:
            # grasping
            if all([abs(sensor.getValue() - 0.5) < 0.01 for sensor in finger_sensors]):
                # I hope that it already grasped an orange.
                shoulder_pan_joint.setPosition(-1.57)
                state = 4
        elif state == 4:
            # shoulder panning
            if abs(shoulder_lift_sensor.getValue() - (-1.57)) < 0.01:
                for finger_joint in finger_joints:
                    finger_joint.setPosition(finger_joint.getMinPosition())
                state = 5
        elif state == 5:
            # releasing
            if all([abs(sensor.getValue() - joint.getMinPosition()) < 0.01
                    for sensor, joint in zip(finger_sensors, finger_joints)]):
                shoulder_pan_joint.setPosition(0)
                state = 6


if __name__ == '__main__':
    main()
