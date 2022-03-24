#!/usr/bin/env python3
import rospy
from rpbi.tf_interface import TfInterface
from ik_ros.msg import TracIKProblem
from sensor_msgs.msg import JointState
from custom_srvs.custom_srvs import ToggleService

class Node:

    def __init__(self):
        rospy.init_node('trac_ik_set_problem_node')
        self.trac_ik_problem = TracIKProblem()
        self.tf = TfInterface()
        self.parent_frame_id = rospy.get_param('~parent_frame_id')
        self.child_frame_id = rospy.get_param('~child_frame_id')
        self.pub = rospy.Publisher('ik', TracIKProblem, queue_size=10)
        hz = rospy.get_param('~hz', 50)
        self.dt = 1.0/float(hz)
        rospy.Subscriber('qinit', JointState, self.joint_state_callback)
        self.timer = None
        ToggleService('toggle_trac_ik_set_problem', self.start, self.stop)
        if rospy.get_param('~start_on_init', False):
            self.start()

    def start(self):
        if self.timer is None:
            self.timer = rospy.Timer(rospy.Duration(self.dt), self.loop)
            success = True
            message = "started trac_ik_set_problem"
        else:
            success = False
            message = 'tried to start trac_ik_set_problem but it is already running'
        return success, message

    def stop(self):
        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None
            success = True
            message = "stopped trac_ik_set_problem"
        else:
            success = False
            message = 'tried to stop trac_ik_set_problem but it is not running'
        return success, message

    def joint_state_callback(self, msg):
        self.trac_ik_problem.qinit = msg

    def loop(self, event):
        tf = self.tf.get_tf_msg(self.parent_frame_id, self.child_frame_id)
        if tf:
            self.trac_ik_problem.goal = tf.transform
            self.trac_ik_problem.header.stamp = rospy.Time.now()
            self.pub.publish(self.trac_ik_problem)

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
