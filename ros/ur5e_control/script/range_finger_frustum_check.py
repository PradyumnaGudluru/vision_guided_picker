import numpy as np

import rospy
from sensor_msgs.msg import Image


def cb_depth_image(msg):
    depth_img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, -1)
    print(np.min(depth_img), np.max(depth_img), np.std(depth_img))


def main():
    rospy.init_node('frustum_check')

    range_finder_sub = rospy.Subscriber('/robot_44327_rabbit/range_finder/range_image',
                                        Image, cb_depth_image)

    rospy.spin()


if __name__ == '__main__':
    main()
