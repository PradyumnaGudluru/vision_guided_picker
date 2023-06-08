# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Joint state publisher."""

import rospy
from sensor_msgs.msg import JointState


class JointStatePublisher(object):
    """Publish as a ROS topic the joint state."""

    jointNames = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint',
        'palm_finger_1_joint',
        'finger_1_joint_1',
        'finger_1_joint_2',
        'finger_1_joint_3',
        'palm_finger_2_joint',
        'finger_2_joint_1',
        'finger_2_joint_2',
        'finger_2_joint_3',
        'finger_middle_joint_1',
        'finger_middle_joint_2',
        'finger_middle_joint_3',
    ]

    def __init__(self, robot, joint_prefix, node_name):
        """Initialize the motors, position sensors and the topic."""
        self.robot = robot
        self.jointPrefix = joint_prefix
        self.motors = []
        self.sensors = []
        self.timestep = int(robot.getBasicTimeStep())
        self.last_joint_states = None
        self.previousTime = 0
        self.previousPosition = []

        for name in JointStatePublisher.jointNames:
            self.motors.append(robot.getDevice(name))
            self.sensors.append(robot.getDevice(name + '_sensor'))
            self.sensors[-1].enable(self.timestep)
            self.previousPosition.append(0)

        self.publisher = rospy.Publisher(node_name + 'joint_states', JointState, queue_size=1)

    def publish(self):
        """Publish the 'joint_states' topic with up to date value."""
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "From simulation state data"
        msg.name = [s + self.jointPrefix for s in JointStatePublisher.jointNames]
        msg.position = []
        time_difference = self.robot.getTime() - self.previousTime

        for i in range(len(self.sensors)):
            value = self.sensors[i].getValue()
            msg.position.append(value)
            msg.velocity.append((value - self.previousPosition[i]) / time_difference if time_difference > 0 else 0.0)
            self.previousPosition[i] = value

        msg.effort = [0] * 6
        self.publisher.publish(msg)
        self.last_joint_states = msg
        self.previousTime = self.robot.getTime()
