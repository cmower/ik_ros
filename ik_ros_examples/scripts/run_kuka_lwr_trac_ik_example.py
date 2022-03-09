#!/usr/bin/env python3
import sys
import rospy
from sensor_msgs.msg import JointState
from ik_ros.msg import TracIKProblem
from rpbi.tf_interface import TfInterface

class Node:


    def __init__(self):
        rospy.init_node('run_lwr_trac_ik_example')
        self.tf = TfInterface()
        self.pub = rospy.Publisher('ik', TracIKProblem, queue_size=10)
        self.trac_ik_problem = TracIKProblem(qinit = rospy.wait_for_message('rpbi/kuka_lwr/joint_states', JointState))
        rospy.Subscriber('rpbi/kuka_lwr/joint_states', JointState, self.callback)
        rospy.Timer(rospy.Duration(1.0/float(rospy.get_param('~hz', 100))), self.loop)

    def callback(self, msg):
        self.trac_ik_problem.qinit = msg

    def loop(self, event):
        tf = self.tf.get_tf_msg('rpbi/kuka_lwr/base', 'figure_eight')
        if tf:
            self.trac_ik_problem.goal = tf
            self.trac_ik_problem.header.stamp = rospy.Time.now()
            self.pub.publish(self.trac_ik_problem)

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
