#!/usr/bin/env python3
import sys
import rospy
from sensor_msgs.msg import JointState
from ik_ros.msg import EXOTicaProblem

class Node:


    def __init__(self):
        rospy.init_node('run_lwr_exotica_example')
        self.pub = rospy.Publisher('ik', EXOTicaProblem, queue_size=10)
        self.previous_solutions = []
        self.start_state = rospy.wait_for_message('rpbi/kuka_lwr/joint_states', JointState)
        rospy.Subscriber('rpbi/kuka_lwr/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('joint_states/target', JointState, self.target_joint_state_callback)
        rospy.Timer(rospy.Duration(1.0/float(rospy.get_param('~hz', 100))), self.loop)

    def joint_state_callback(self, msg):
        self.start_state = msg

    def target_joint_state_callback(self, msg):
        self.previous_solutions.append(msg)
        if len(self.previous_solutions) == 2:
            self.previous_solutions.pop(0)

    def loop(self, event):
        self.pub.publish(EXOTicaProblem(start_state=self.start_state, previous_solutions=self.previous_solutions))

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
