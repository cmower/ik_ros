#!/usr/bin/env python3
import sys
import math
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from ros_pybullet_interface.utils import resolve_joint_position_order
from ik_ros.srv import JointNameOrder

qinit_deg = [0, 30, 0, 90, 0, -30, 0]
qinit = [math.radians(q) for q in qinit_deg]

class Node:

    def __init__(self):
        rospy.init_node('trac_ik_example_node')
        self.pub = rospy.Publisher('setup', Float64MultiArray, queue_size=10)
        self.transform = None
        self.joint_states = None
        rospy.Subscriber('transform', Float64MultiArray, self.transform_callback)
        hz = rospy.get_param('~hz', 50)
        rospy.Timer(rospy.Duration(1.0/float(hz)), self.main_loop)

    def transform_callback(self, msg):
        self.transform = list(msg.data)

    def main_loop(self, event):
        if self.transform is None: return
        self.pub.publish(Float64MultiArray(data=self.transform+qinit))

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
