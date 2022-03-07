#!/usr/bin/env python3
import rospy
from ik_ros.msg import TracIKProblem
from rpbi.tf_interface import TfInterface

ndof = 7  # no. degrees of freedom for Kuka LWR

class Node:


    def __init__(self):
        rospy.init_node('run_lwr_trac_ik_example')
        self.tf = TfInterface()
        self.pub = rospy.Publisher('ik', TracIKProblem, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/float(rospy.get_param('~hz'))))


    def loop(self, event):
        tf = self.tf.get_tf_msg('world', 'figure_eight')
        if tf:
            self.pub.publish(TracIKProblem(qinit=[0.0]*ndof, goal=tf))


    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
