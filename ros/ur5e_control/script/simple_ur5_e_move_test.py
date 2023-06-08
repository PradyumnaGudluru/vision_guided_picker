import numpy as np

import rospy
from std_msgs.msg import String
from webots_ros.srv import set_float


def main():
    """
    it expects the Webots "ros" controller is used for a UR5e robot.

    and it also assumes that there is only one robot that uses the Webots "ros" controller
    """
    rospy.init_node('ur5_controller', anonymous=True)

    # assuming there is only one robot, you need take only one message!
    model_name_msg = rospy.wait_for_message("/model_name", String)

    # just follows Webots "ros" controller naming convention
    shoulder_pan = rospy.ServiceProxy(f'/{model_name_msg.data}/shoulder_pan_joint/set_position', set_float)
    shoulder_lift = rospy.ServiceProxy(f'/{model_name_msg.data}/shoulder_lift_joint/set_position', set_float)
    elbow = rospy.ServiceProxy(f'/{model_name_msg.data}/elbow_joint/set_position', set_float)
    wrist1 = rospy.ServiceProxy(f'/{model_name_msg.data}/wrist_1_joint/set_position', set_float)
    wrist2 = rospy.ServiceProxy(f'/{model_name_msg.data}/wrist_2_joint/set_position', set_float)
    wrist3 = rospy.ServiceProxy(f'/{model_name_msg.data}/wrist_3_joint/set_position', set_float)

    # [-np.pi / 2, np.pi / 2, 0, 0, np.pi / 2, 0] is the initial pose that I want?
    shoulder_pan(np.pi / 2)
    shoulder_lift(-np.pi / 2)
    elbow(0)
    wrist2(np.pi / 2)

    rospy.spin()


if __name__ == '__main__':
    main()
