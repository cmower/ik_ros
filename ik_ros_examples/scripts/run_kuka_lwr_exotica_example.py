#!/usr/bin/env python3
import sys
import rospy
from rpbi.tf_interface import TfInterface
from sensor_msgs.msg import JointState
from ik_ros.msg import EXOTicaProblem
from std_srvs.srv import SetBool
from ros_pybullet_interface.msg import CalculateInverseKinematicsProblem
from ros_pybullet_interface.srv import ResetEffState, ResetEffStateRequest

class Node:


    def __init__(self):
        rospy.init_node('run_lwr_exotica_example')
        self.tf = TfInterface()
        self.pub = rospy.Publisher('ik', EXOTicaProblem, queue_size=10)
        rospy.loginfo('moving robot to start position')
        self.move_to_robot_to_start()
        rospy.loginfo('robot at starting position')
        self.exotica_problem = EXOTicaProblem(start_state = rospy.wait_for_message('rpbi/kuka_lwr/joint_states', JointState))
        self.start_fig8()
        rospy.loginfo('started figure 8')
        rospy.Subscriber('rpbi/kuka_lwr/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('joint_states/target', JointState, self.target_joint_state_callback)
        rospy.Timer(rospy.Duration(1.0/float(rospy.get_param('~hz', 100))), self.loop)
        rospy.loginfo('started sending joint target commands to robot')

    def start_fig8(self):
        srv_name = 'toggle_fig8'
        rospy.wait_for_service(srv_name)
        handle = rospy.ServiceProxy(srv_name, SetBool)
        handle(False)  # ensure fig8 is reset
        handle(True)

    def move_to_robot_to_start(self):

        # Get target
        timeout = 10.0  # secs
        start_time = rospy.Time.now()
        while (rospy.Time.now()-start_time).to_sec() < timeout:
            pos, rot = self.tf.get_tf('rpbi/world', 'figure_eight_base')
            if pos is not None: break
        else:
            rospy.logerr('could not get transform')
            sys.exit(0)

        # Setup problem
        problem = CalculateInverseKinematicsProblem()
        problem.link_name = 'lwr_arm_7_link'
        problem.targetPosition = pos
        problem.targetOrientation = rot

        # Setup request
        duration = 3.0
        req = ResetEffStateRequest(problem=problem, duration=duration)

        # Move robot
        srv_name = 'rpbi/kuka_lwr/move_to_eff_state'
        rospy.wait_for_service(srv_name)
        move = rospy.ServiceProxy(srv_name, ResetEffState)
        resp = move(req)


    def joint_state_callback(self, msg):
        self.exotica_problem.start_state = msg

    def target_joint_state_callback(self, msg):
        self.exotica_problem.previous_solutions.append(msg)
        if len(self.exotica_problem.previous_solutions) == 2:
            self.exotica_problem.previous_solutions.pop(0)

    def loop(self, event):
        self.exotica_problem.header.stamp = rospy.Time.now()
        self.pub.publish(self.exotica_problem)

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
