#! /usr/bin/env python
import copy
import os
import threading
import time
import sys

import numpy as np
from scipy.spatial.transform import Rotation

import rospy
import rospkg

import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommand

import std_msgs.msg
import sensor_msgs.msg

# you may need `sudo apt install ros-noetic-webots-ros`
from webots_ros.srv import set_float
from webots_ros.msg import Float64Stamped

import torch
import torchvision

import torchvision.models.detection.faster_rcnn as faster_rcnn
import torchvision.models.detection.mask_rcnn as mask_rcnn

import moveit_commander
from geometry_msgs.msg import Pose


def get_model_instance_segmentation(num_classes):
    """create a custom mask r-cnn model for a different number of classes

    which classes does the COCO dataset?
    """
    # load an instance segmentation model which is pre-trained on COCO
    model = torchvision.models.detection.maskrcnn_resnet50_fpn(pretrained=True)

    # get number of input features for the classifier
    in_features = model.roi_heads.box_predictor.cls_score.in_features

    # replace the pre-trained head with a new one
    model.roi_heads.box_predictor = faster_rcnn.FastRCNNPredictor(in_features, num_classes)

    # now get the number of input features for the mask classifier
    in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
    hidden_layer = 256
    # and replace the mask predictor with a new one
    model.roi_heads.mask_predictor = mask_rcnn.MaskRCNNPredictor(in_features_mask, hidden_layer, num_classes)

    return model


def get_robot_names(num_robots):
    """
    get robot names

    Returns:
        set[str]: the name of robots
    """
    robot_names = set()

    def callback_(msg):
        robot_names.add(msg.data)

    sub = rospy.Subscriber('/model_name', std_msgs.msg.String, callback_)

    while not rospy.is_shutdown():
        if len(robot_names) > num_robots - 1:
            sub.unregister()
            return robot_names


class OrangeSegmentationThread(threading.Thread):
    def __init__(self, queue, cv):
        super().__init__(daemon=True)
        self.queue = queue
        self.cv = cv

        # this thread tells the main thread whether it is processing
        self.processing = False

        # the main thread tells whether this thread should stop
        self.running = True

        self.latest_mask = None
        self.depth_image = None
        self.x = None
        self.y = None
        self.angle = None

    def run(self):
        mask_pub = rospy.Publisher('orange_mask', sensor_msgs.msg.Image, queue_size=1)

        # we're using 2-class (orange and background) model
        model = get_model_instance_segmentation(2)

        # load pre-trained parameters
        ros_pack = rospkg.RosPack()
        pack_path = ros_pack.get_path('ur5e_control')
        param_path = os.path.join(pack_path, 'orange_mask_rcnn_10_epochs.pth')

        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        model.load_state_dict(torch.load(param_path))
        model = model.to(device)
        model.eval()

        while True:
            with self.cv:
                # running, processing, queue are shared variables!
                while self.running and len(self.queue) < 1:
                    self.cv.wait()

                if not self.running:
                    return

                # here, self.running is True and self.queue is not empty!
                img, depth_img, x, y, angle = self.queue.pop(0)
                self.processing = True

            # processing img
            with torch.no_grad():
                # ROS uses (height, width, 4(=rgba)) format for images
                # pytorch requires it to be (3, height, width)
                img = np.swapaxes(img[:, :, :3], 0, 2)
                img = np.swapaxes(img, 1, 2)

                # also in [0, 1] range
                # why img was not writable?
                img = (torch.Tensor(np.array(img)).float() / 255).to(device)

                # input shape (1, 3, height, width)
                pred, = model(torch.unsqueeze(img, dim=0))

                if pred['masks'].shape[0] > 0:
                    # what should I do if it has multiple instances?

                    # mask is also in [0, 1] range
                    # convert to [0, 255] range
                    mask = (pred['masks'][0].detach().cpu().numpy() * 255).astype(np.uint8)
                else:
                    mask = np.zeros((1, 320, 480), dtype=np.uint8)

                with self.cv:
                    self.latest_mask = mask
                    self.depth_image = depth_img
                    self.x = x
                    self.y = y
                    self.angle = angle

                # I'm not sure what is a right way to construct a ROS Image message
                # but it seems to work for now
                # supported encodings are listed here?
                # (http://docs.ros.org/en/jade/api/sensor_msgs/html/image__encodings_8h_source.html)
                mask_msg = sensor_msgs.msg.Image(height=mask.shape[1], width=mask.shape[2],
                                                 data=mask.tobytes(), encoding='mono8')

                mask_pub.publish(mask_msg)

            with self.cv:
                self.processing = False


def main():
    depth_img = None
    x, y, angle = None, None, None
    queue = []

    cv = threading.Condition()
    orange_seg_thread = OrangeSegmentationThread(queue, cv)
    orange_seg_thread.start()

    def cb_depth_image(msg):
        """
        just capture of the last depth image

        Args:
            msg: ROS Image message

        Returns:
            None:
        """
        nonlocal depth_img
        # print(msg.height, msg.width)
        depth_img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, -1)

    def cb_image(msg):
        """
        since it's like that the processing images is much slower
        than the rate of incoming images it does a flow control

        Args:
            msg: ROS Image message

        Returns:
            None:
        """
        with cv:
            if orange_seg_thread.processing or len(queue) > 0 or depth_img is None:
                return

            # 1. a segmentation thread is not processing
            # 2. queue is empty
            # 3. depth image is also available
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, -1, 4)
            queue.append([img, depth_img[...], x, y, angle])

            # some state is updated. notify a segmentation thread
            cv.notifyAll()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('orange_segmentation')
    move_group = moveit_commander.MoveGroupCommander('arm')

    # here the joint values of 'home' are [0, -1, 2, -2.57, -1.57, 0]
    move_group.set_named_target('home')
    move_group.go(wait=True)

    # t = 1.0
    # joint_goal = [0, -t, t * 2, -t - np.pi / 2, -np.pi / 2, 0]
    # move_group.go(joint_goal, wait=True)

    ac = actionlib.ActionClient('/gripper_controller/gripper_action', GripperCommandAction)
    ac.wait_for_action_server_to_start()

    xy_stage_name, = get_robot_names(1)

    range_finder_sub = rospy.Subscriber(f'/{xy_stage_name}/range_finder/range_image',
                                        sensor_msgs.msg.Image, cb_depth_image)
    camera_sub = rospy.Subscriber(f'/{xy_stage_name}/camera/image',
                                  sensor_msgs.msg.Image, cb_image)

    def cb_x(msg):
        nonlocal x
        x = msg.data

    def cb_y(msg):
        nonlocal y
        y = msg.data

    def cb_rot(msg):
        nonlocal angle
        angle = msg.data

    x_sub = rospy.Subscriber(f'/{xy_stage_name}/x_position_sensor/value', Float64Stamped, cb_x)
    y_sub = rospy.Subscriber(f'/{xy_stage_name}/y_position_sensor/value', Float64Stamped, cb_y)
    rot_sub = rospy.Subscriber(f'/{xy_stage_name}/rotation_sensor/value', Float64Stamped, cb_rot)

    x_proxy = rospy.ServiceProxy(f'/{xy_stage_name}/x_motor/set_position', set_float)
    y_proxy = rospy.ServiceProxy(f'/{xy_stage_name}/y_motor/set_position', set_float)
    rot_proxy = rospy.ServiceProxy(f'/{xy_stage_name}/rotational_motor/set_position', set_float)

    rate = rospy.Rate(5)

    state = 0
    direction = 1
    angle_direction = 1

    target_x, target_y, target_angle = None, None, None

    camera_frame = Rotation.from_rotvec((0, 0, np.pi / 2)) * Rotation.from_rotvec((0, 0.5, 0))

    def locate_orange(current_mask, matching_depth, x, y, angle):
        """

        locate an orange with respect to the xy stage frame
        with a given xy stage coordinates (x, y), camera hinge joint angle (angle),
            instance mask, and range-finder image

        note that 'camera/image' and 'range_finder/range_image' topics may not be synchronized
        if it is the case, how can I mitigate this issue?

        Args:
            current_mask: instance segmentation mask
            matching_depth: matching z-depth
            x:
            y:
            angle:

        Returns:
            (float, float, float): the location of an orange
        """
        vs, hs = np.where(current_mask)
        m_v, m_h = int(np.mean(vs)), int(np.mean(hs))
        # print('depth:', depth_img[m_v // 2, m_h // 2])

        # radius of an orange is 0.05!
        depth = matching_depth[m_v // 2, m_h // 2] + 0.05

        # note that the orientation of the range_finder
        # x is along the depth
        # y is to the left
        # z is to the top
        rx = depth
        rz = ((359 - m_v) - 179.5) / 480 * 0.785 * depth
        ry = -(m_h - 239.5) / 480 * 0.785 * depth

        trans1 = Rotation.from_rotvec((0, 0, angle)) * camera_frame
        rx2, ry2, rz2 = trans1.apply([rx, ry, rz])
        rx2 += (x + 0.5)
        ry2 += y
        rz2 += 1

        return rx2, ry2, rz2

    ox, oy, oz = None, None, None

    while not rospy.is_shutdown():
        rate.sleep()

        if state == 0:
            # found an orange?
            # if found, try to position the camera to there -> state 1
            with cv:
                mask = orange_seg_thread.latest_mask
                m_depth = orange_seg_thread.depth_image
                x = orange_seg_thread.x
                y = orange_seg_thread.y
                angle = orange_seg_thread.angle
                if mask is not None and angle is not None:
                    mask = mask[0] > 128
                    if np.sum(mask) > 750:
                        rx2, ry2, rz2 = locate_orange(mask, m_depth, x, y, angle)
                        print(f'c_x = {x:.2f}', f'c_y = {y:.2}', f'c_angle = {angle % np.pi * 2:.2f}')
                        print(f'o_x = {rx2:.2f}, o_y = {ry2:.2f}, o_z = {rz2:.2f}')

                        if abs(rx2) < 4 and rz2 > 0.1:
                            # move the carriage to here!
                            # align the camera and an orange along the axis
                            target_x = rx2 - 0.5

                            if ry2 > 0:
                                # target_y = 0.3
                                if angle % (np.pi * 2) > np.pi:
                                    target_angle = np.ceil(angle / (2 * np.pi)) * np.pi * 2
                                else:
                                    target_angle = np.floor(angle / (2 * np.pi)) * np.pi * 2
                            else:
                                # target_y = -0.3
                                if (angle - np.pi) % (np.pi * 2) > np.pi:
                                    target_angle = np.ceil(angle / (2 * np.pi)) * np.pi * 2 + np.pi
                                else:
                                    target_angle = np.floor(angle / (2 * np.pi)) * np.pi * 2 + np.pi

                            x_proxy(target_x)
                            # y_proxy(target_y)
                            rot_proxy(target_angle)

                            state = 1
                            print('move to', target_x, target_angle)
                            continue
            if angle is None or x is None:
                continue

            if angle > 0.3:
                angle_direction = -1
            elif angle < -0.3:
                angle_direction = 1

            rot_proxy(angle + 0.1 * angle_direction)

            if x > 4:
                direction = -1
            elif x < -4:
                direction = 1

            x_proxy(x + 0.1 * direction)
        elif state == 1:
            # if a camera movement is finished? -> state 2
            if abs(angle - target_angle) < 0.005 and abs(x - target_x) < 0.005:
                state = 2
        elif state == 2:
            for _ in range(5):
                rate.sleep()

            # camera must be already approximately aligned and have been waiting for ~ 0.5 second
            # take a picture and -> state 3
            try:
                mask = orange_seg_thread.latest_mask
                mask = mask[0] > 128
                m_depth = orange_seg_thread.depth_image
                x = orange_seg_thread.x
                y = orange_seg_thread.y
                angle = orange_seg_thread.angle
                ox, oy, oz = locate_orange(mask, m_depth, x, y, angle)
                if oz < 0.1:
                    print('too low orange!')
                    state = 0
                    continue
                print(x, y, ox, oy, oz)
            except ValueError:
                state = 0
                continue

            # robotic arm is -0.35 meter off from the camera
            target_x = ox + 0.35
            target_y = oy - 0.53 if oy > 0 else oy + 0.53

            x_proxy(target_x)
            y_proxy(target_y)
            state = 3
        elif state == 3:
            if abs(x - target_x) < 0.005 and abs(y - target_y) < 0.005:
                state = 4
        elif state == 4:
            waypoints = []

            # First move up (z) and sideways (y)
            w_pose = move_group.get_current_pose().pose
            print('ee_link position', w_pose.position)

            w_pose.position.x = 0.15
            original_y = w_pose.position.y
            original_z = w_pose.position.z

            w_pose.position.y = oy - y
            waypoints.append(copy.deepcopy(w_pose))

            w_pose.position.z -= (0.68 - oz)
            waypoints.append(copy.deepcopy(w_pose))

            # We want the Cartesian path to be interpolated at a resolution of 1 cm
            # which is why we will specify 0.01 as the eef_step in Cartesian
            # translation.  We will disable the jump threshold by setting it to 0.0,
            # ignoring the check for infeasible jumps in joint space, which is sufficient
            # for this tutorial.
            (plan, fraction) = move_group.compute_cartesian_path(
                waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold

            print(type(plan), type(fraction))
            move_group.execute(plan, wait=True)

            goal = GripperCommandGoal()

            # Released 0.05 <-> 0.5 Grasp
            goal.command = GripperCommand(position=0.5, max_effort=-1)

            gh = ac.send_goal(goal)
            ac.wait_for_server()

            waypoints = []

            w_pose.position.z = original_z
            waypoints.append(copy.deepcopy(w_pose))

            # Second move forward/backwards in (x)
            w_pose.position.x = 0.5
            w_pose.position.y = original_y
            waypoints.append(copy.deepcopy(w_pose))

            (plan, fraction) = move_group.compute_cartesian_path(
                waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold

            print(type(plan), type(fraction))
            move_group.execute(plan, wait=True)

            goal = GripperCommandGoal()

            # Released 0.05 <-> 0.5 Grasp
            goal.command = GripperCommand(position=0.05, max_effort=-1)

            gh = ac.send_goal(goal)
            ac.wait_for_server()

            for _ in range(5):
                rate.sleep()

            y_proxy(0)
            state = 5

            # move_group.set_named_target('home')
            # move_group.go(wait=True)

            # move_group.set_named_target('reset')
            # move_group.go(wait=True)
            # move_group.stop()
        elif state == 5:
            if abs(y) < 0.01:
                state = 0


if __name__ == '__main__':
    main()
