import numpy as np
from scipy.linalg import expm

from controller import Robot


def screw_axis(omega=None, v=None):
    """
    screw axis is a twist of (1) an unit omega or (2) zero omega and unit v?

    (omega, v) -> twist in se(4)
    Args:
        omega:
        v:

    Returns:
        (se(4))
    """
    s = np.zeros((4, 4))
    
    if omega:
        wx, wy, wz = omega
        s[:3, :3] = [[0, -wz, wy],
                     [wz, 0, -wx],
                     [-wy, wx, 0]]
    if v:
        s[:3, -1] = v
    
    return s


def matw_from_vecw(w):
    """
    w in R^3 -> [w] in so(3)
    Args:
        w:

    Returns:
        (so(3))
    """
    wx, wy, wz = w

    return np.asarray([[0, -wz, wy],
                       [wz, 0, -wx],
                       [-wy, wx, 0]], dtype=np.float32)


def r_from_vecw(w):
    """
    equivalent to following

    from scipy.spatial.transform.rotation import Rotation


    Rotation.from_rotvec(w).as_matrix()

    Args:
        w: rotvec in R^3

    Returns:
        (SO(3))
    """
    return expm(matw_from_vecw(w))


def pose(w, p):
    """
    Args:
        w:
        p:

    Returns:
        (SE(4))
    """
    s = np.eye(4)
    s[:3, :3] = r_from_vecw(w)
    s[:3, -1] = p
    
    return s


def end_locations(carrier_ths, joints):
    r = np.zeros(18)
    
    # joint screw axes
    s1 = screw_axis([0, 0, 1])
    s2 = screw_axis([1, 0, 0])
    s3 = screw_axis([0, 1, 0], [0, 0, 0.75])
    
    # end effector pose at the home position
    m = np.asarray([[1, 0, 0, 0.75],
                    [0, 1, 0, 0],
                    [0, 0, 1, 1],
                    [0, 0, 0, 1]])
    
    # 6 scalar constrains for outer carrier
    for i, (th, (x_th, y_th)) in enumerate(zip(carrier_ths[::2], joints[::2])):
        r[6 * i: 6 * i + 3] = (expm(th * s1) @ expm(x_th * s2) @ expm(y_th * s3) @ m)[:3, -1]
    
    # end effector pose at the home position
    m = np.asarray([[1, 0, 0, 0.6],
                    [0, 1, 0, 0],
                    [0, 0, 1, 1],
                    [0, 0, 0, 1]])
    
    # joint screw axis for y-axis of uj
    s3 = screw_axis([0, 1, 0], [0, 0, 0.6])
    
    # 6 scalar constrains for inner carrier
    for i, (th, (x_th, y_th)) in enumerate(zip(carrier_ths[1::2], joints[1::2])):
        r[6 * i + 3: 6 * i + 6] = (expm(th * s1) @ expm(x_th * s2) @ expm(y_th * s3) @ m)[:3, -1]
            
    return r


def anchor_locations(platform_pose):
    anchors = np.zeros(18)
    
    # motion platform pose
    w, p = platform_pose[:3], platform_pose[3:]
    pp = pose(w, p)
    
    # 3 ball joint poses
    r = 0.5
    bs = np.asarray([[r * np.cos(angle), r * np.sin(angle), 0, 1] for angle
                     in np.asarray([1, 3, 5]) * (np.pi / 3)])
    
    for i, b in enumerate(bs):
        anchors[6 * i + 0: 6 * i + 3] = anchors[6 * i + 3: 6 * i + 6] = (pp @ b)[:3]
    
    return anchors


def d_kinematic_constraints(carrier_ths, joints, platform_pose):
    """
    Jacobian?

    Args:
        carrier_ths (numpy.ndarray): shape=(6,)
        joints (numpy.ndarray): shape=(6, 2) for 6 universal joints
        platform_pose (numpy.ndarray): shape=(6,)
    """
    diff = np.zeros((18, 18))

    # motion platform pose
    w, p = platform_pose[:3], platform_pose[3:]
    pp = pose(w, p)
    
    # 3 ball joint poses
    r = 0.5
    bs = np.asarray([[r * np.cos(angle), r * np.sin(angle), 0, 1] for angle
                     in np.asarray([1, 3, 5]) * (np.pi / 3)])
    
    # end effector pose at the home position
    m = np.asarray([[1, 0, 0, 0.75],
                    [0, 1, 0, 0],
                    [0, 0, 1, 1],
                    [0, 0, 0, 1]])

    # joint screw axes
    s1 = screw_axis([0, 0, 1])
    s2 = screw_axis([1, 0, 0])
    s3 = screw_axis([0, 1, 0], [0, 0, 0.75])
    
    # 6 scalar constrains for outer carrier
    for i, (th, (x_th, y_th), b) in enumerate(zip(carrier_ths[::2], joints[::2], bs)):
        b = expm(y_th * s3) @ m
        s1m = expm(th * s1)
        s2m = expm(x_th * s2)
        
        diff[6 * i: 6 * i + 3, 6 * i + 0] = (s1 @ s1m @ s2m @ b)[:3, -1]
        diff[6 * i: 6 * i + 3, 6 * i + 1] = (s1m @ s2 @ s2m @ b)[:3, -1]
        diff[6 * i: 6 * i + 3, 6 * i + 2] = (s1m @ s2m @ s3 @ b)[:3, -1]

    # end effector pose at the home position
    m = np.asarray([[1, 0, 0, 0.6],
                    [0, 1, 0, 0],
                    [0, 0, 1, 1],
                    [0, 0, 0, 1]])
    
    # joint screw axis for y-axis of uj
    s3 = screw_axis([0, 1, 0], [0, 0, 0.6])
    
    # 6 constraints for inner carrier
    for i, (th, (x_th, y_th), b) in enumerate(zip(carrier_ths[1::2], joints[1::2], bs)):
        b = expm(y_th * s3) @ m
        s1m = expm(th * s1)
        s2m = expm(x_th * s2)
                
        diff[6 * i + 3: 6 * i + 6, 6 * i + 3] = (s1 @ s1m @ s2m @ b)[:3, -1]
        diff[6 * i + 3: 6 * i + 6, 6 * i + 4] = (s1m @ s2 @ s2m @ b)[:3, -1]
        diff[6 * i + 3: 6 * i + 6, 6 * i + 5] = (s1m @ s2m @ s3 @ b)[:3, -1]
    
    return diff
    

def inverse_kinematics(target_pose, joints):
    ths = joints[::3]
    uv_joints = np.asarray(joints).reshape(6, -1)[:, 1:]
    
    for _ in range(1):
        dth = d_kinematic_constraints(ths,
                                      uv_joints,
                                      target_pose)
        diff = anchor_locations(target_pose) - end_locations(ths, uv_joints)
        dparams = np.linalg.solve(dth, diff)
        
        new_ths = np.asarray(ths) + dparams[::3]
        new_joints = uv_joints + dparams.reshape(6, -1)[:, 1:]
    
        new_diff = anchor_locations(target_pose) - end_locations(new_ths, new_joints.reshape(6, -1))
    
        ths = new_ths
        uv_joints = new_joints
    
    x = np.zeros(18)
    x[::3] = ths
    x[1::3] = uv_joints[:, 0]
    x[2::3] = uv_joints[:, 1]
    
    return x


def main():
    time_step = 32
    robot = Robot()

    motors = [robot.getDevice(f'motor{x}') for x in range(6)]
    
    for motor in motors:
        motor.getPositionSensor().enable(time_step)
    
    ths = [-0.2915210927368512, 2.6680147347983985,
           1.802874009656344, 4.762409837191594,
           3.8972691120495395, 6.856804939584789]

    uv_joints = [-0.68141, -0.68801, 0.69412, -0.67513] * 3
    joints = np.zeros(18)
    joints[::3] = ths
    joints[1::3] = uv_joints[::2]
    joints[2::3] = uv_joints[1::2]
    
    th = 0
    amp = 0.2
    
    while robot.step(time_step) != -1:
        ths = [m.getPositionSensor().getValue() for m in motors]
        platform_pose = [0, 0, 0, 0.15 * np.sin(th), 0, 0.6]
        
        target_joints = inverse_kinematics(platform_pose, joints)
        
        for p, m in zip(target_joints[::3], motors):
            m.setPosition(p)

        th += 0.1
        joints = target_joints


if __name__ == '__main__':
    main()
