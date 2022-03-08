#!/usr/bin/env python3
import sys
import rospy
from sensor_msgs.msg import JointState
from ik_ros.msg import EXOTicaProblem

class Node:


    def __init__(self):
        rospy.init_node('run_lwr_exotica_example')
        self.pub = rospy.Publisher('ik', EXOTicaProblem, queue_size=10)
        self.start_state = rospy.wait_for_message('rpbi/kuka_lwr/joint_states', JointState)
        rospy.Subscriber('rpbi/kuka_lwr/joint_states', JointState, self.callback)
        rospy.Timer(rospy.Duration(1.0/float(rospy.get_param('~hz', 100))), self.loop)

    def callback(self, msg):
        self.start_state = msg

    def loop(self, event):
        self.pub.publish(EXOTicaProblem(start_state=self.start_state))

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
