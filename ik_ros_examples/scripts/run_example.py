#!/usr/bin/env python3
import sys
import time
import rospy
from rpbi.tf_interface import TfInterface
from std_srvs.srv import SetBool
from ros_pybullet_interface.msg import CalculateInverseKinematicsProblem
from ros_pybullet_interface.srv import ResetEffState, ResetEffStateRequest

def get_srv_handle(srv_name, srv_type):
    handle = None
    rospy.wait_for_service(srv_name)
    try:
        handle = rospy.ServiceProxy(srv_name, srv_type)
    except rospy.ServiceException as e:
        rospy.logerr(f"service call failed: {str(e)}")
    return handle

class Node:

    def __init__(self):

        # Initialize ROS node
        rospy.init_node('run_example')

        # Setup tf interface
        self.tf = TfInterface()

        # Get parameters
        self.interface_name = sys.argv[1]
        robot_name = rospy.get_param('~robot_name')
        self.eff_name = rospy.get_param('~eff_name')

        # Get service handles
        self.move_to_eff_state = get_srv_handle(f'rpbi/{robot_name}/move_to_eff_state', ResetEffState)
        self.start_figure_eight = get_srv_handle('toggle_figure_eight', SetBool)
        self.start_ik_setup_node = get_srv_handle(f'ik/setup/{self.interface_name}/toggle', SetBool)
        if self.interface_name != 'pybullet':
            self.start_ik_solver_node = get_srv_handle(f'ik/solver/{self.interface_name}/toggle', SetBool)

    def move_to_start_pose(self):

        # Get eff target
        timeout = 10.0
        start_time = time.time()
        while (time.time()-start_time) < timeout:
            init_eff_pos, init_eff_rot = self.tf.get_tf('rpbi/world', 'figure_eight_base')
            if init_eff_pos is not None:
                break
        else:
            rospy.logerr("could not recieve end-effector target transform")
            sys.exit(0)

        # Setup problem
        problem = CalculateInverseKinematicsProblem()
        problem.link_name = self.eff_name
        problem.targetPosition = init_eff_pos
        problem.targetOrientation = init_eff_rot

        # Setup request
        duration = 3.0
        req = ResetEffStateRequest(problem=problem, duration=duration)

        # Move robot
        self.move_to_eff_state(req)

    def start_figure_eight_motion(self):
        self.start_ik_setup_node(True)
        if self.interface_name != 'pybullet':
            self.start_ik_solver_node(True)
        self.start_figure_eight(True)


def main():
    node = Node()
    node.move_to_start_pose()
    node.start_figure_eight_motion()


if __name__ == '__main__':
    main()
