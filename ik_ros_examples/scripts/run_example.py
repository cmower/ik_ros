#!/usr/bin/env python3
import sys
import time
import rospy
from custom_ros_tools.tf import TfInterface
from geometry_msgs.msg import Transform
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from ik_ros.srv import EXOTicaInfo
from ik_ros.srv import EXOTica
from ik_ros.msg import EXOTicaProblem, EXOTicaSyncTf
from ik_ros.srv import TracIK
from ik_ros.msg import TracIKProblem
from sensor_msgs.msg import JointState
from ros_pybullet_interface.msg import CalculateInverseKinematicsProblem
from ros_pybullet_interface.srv import ResetEffState, ResetEffStateRequest
from ros_pybullet_interface.srv import ResetJointState, ResetJointStateRequest
from custom_ros_tools.ros_comm import get_srv_handler

class Node:

    move_to_start_duration = 5.0

    def __init__(self):

        # Initialize ROS node
        rospy.init_node('run_example')

        # Setup tf interface
        self.tf = TfInterface()

        # Get parameters
        self.interface_name = sys.argv[1]
        self.robot_name = rospy.get_param('~robot_name')
        self.eff_name = rospy.get_param('~eff_name')

        # Get service handles
        self.move_to_eff_state = get_srv_handler(f'rpbi/{self.robot_name}/move_to_eff_state', ResetEffState)
        self.move_to_joint_state = get_srv_handler(f'rpbi/{self.robot_name}/move_to_joint_state', ResetJointState)
        self.start_figure_eight = get_srv_handler('toggle_figure_eight', SetBool)
        self.start_ik_setup_node = get_srv_handler(f'ik/setup/{self.interface_name}/toggle', SetBool)
        self.move_to_start_pose = self.move_to_start_pose_using_pybullet
        self.solve_ik = None
        if self.interface_name != 'pybullet':
            self.start_ik_solver_node = get_srv_handler(f'ik/solver/{self.interface_name}/toggle', SetBool)
            if self.interface_name == 'trac_ik':
                srv_type = TracIK
                self.interface_problem_type = TracIKProblem
            elif self.interface_name == 'exotica':
                srv_type = EXOTica
                self.interface_problem_type = EXOTicaProblem
            self.solve_ik = get_srv_handler(f'ik/solver/{self.interface_name}/solve', srv_type)
            self.move_to_start_pose = self.move_to_start_pose_using_interface

        # Handle real robot
        self.toggle_remapper = None
        self.real_robot = rospy.get_param('real_robot', False)
        if self.real_robot:
            self.toggle_remapper = get_srv_handler('remap_joint_state_to_floatarray/toggle', SetBool)
            self.sync_pybullet_with_real_robot()

    def sync_pybullet_with_real_robot(self):
        duration = 0.2
        self.move_to_joint_state(rospy.wait_for_message('joint_states', JointState), duration)

    def get_start_pose(self):

        timeout = 10.0
        start_time = time.time()
        while (time.time()-start_time) < timeout:
            init_eff_pos, init_eff_rot = self.tf.get_tf('rpbi/world', 'figure_eight_base')
            if init_eff_pos is not None:
                break
        else:
            rospy.logerr("could not recieve end-effector target transform")
            sys.exit(0)

        return init_eff_pos, init_eff_rot

    def move_to_start_pose_using_pybullet(self):

        init_eff_pos, init_eff_rot = self.get_start_pose()

        # Setup problem
        problem = CalculateInverseKinematicsProblem()
        problem.link_name = self.eff_name
        problem.targetPosition = init_eff_pos
        problem.targetOrientation = init_eff_rot

        # Setup request
        duration = self.move_to_start_duration
        req = ResetEffStateRequest(problem=problem, duration=duration)

        # Start remapper
        if self.real_robot:
            self.toggle_remapper(True)

        # Move robot
        self.move_to_eff_state(req)


    def setup_exotica_problem(self):

        init_eff_pos, init_eff_rot = self.get_start_pose()

        # Get EXOTica info
        rospy.wait_for_service('exotica_info', timeout=10.0)
        get_exotica_info = rospy.ServiceProxy('exotica_info', EXOTicaInfo)
        exotica_info = get_exotica_info()

        # Setup problem
        problem = EXOTicaProblem()
        problem.task_map_names = exotica_info.task_map_names
        problem.task_map_goals = [
            Float64MultiArray(data=[0.0]*nrho)
            for nrho in exotica_info.task_map_nrho
        ]
        problem.start_state = rospy.wait_for_message(f'rpbi/{self.robot_name}/joint_states', JointState)
        problem.previous_solutions = [problem.start_state]

        problem.sync_tf = []
        for s in rospy.get_param('ik_setup/sync_tf_to_exotica_object', []):
            tf_parent_frame, tf_child_frame, exo_parent_frame, exo_child_frame = s.split(' ')
            if tf_child_frame == 'figure_eight':
                tf_child_frame += '_base'
            if exo_parent_frame == "''":
                exo_parent_frame = ''
            sync_tf = EXOTicaSyncTf(
                tf_parent=tf_parent_frame,
                tf_child=tf_child_frame,
                exo_parent=exo_parent_frame,
                exo_child=exo_child_frame,
            )
            pos, rot = self.tf.get_tf(tf_parent_frame, tf_child_frame)
            sync_tf.transform = pos.tolist()+rot.tolist()
            problem.sync_tf.append(sync_tf)

        return problem

    def setup_trac_ik_problem(self):
        problem = TracIKProblem()
        default_params = {'bx': 1e-5, 'by': 1e-5, 'bz': 1e-5, 'brx': 1e-3, 'bry': 1e-3, 'brz': 1e-3}
        for k, dp in default_params.items():
            param_name = '~init_'+k
            if rospy.has_param(param_name):
                p = rospy.get_param(param_name)
            else:
                p = dp
            setattr(problem, k, p)
        problem.qinit = rospy.wait_for_message(f'rpbi/{self.robot_name}/joint_states', JointState)
        init_eff_pos, init_eff_rot = self.get_start_pose()
        problem.goal = Transform()
        for i, d in enumerate('xyz'):
            setattr(problem.goal.translation, d, init_eff_pos[i])
            setattr(problem.goal.rotation, d, init_eff_rot[i])
        problem.goal.rotation.w = init_eff_rot[3]
        return problem


    def move_to_start_pose_using_interface(self):
        setup_problem = getattr(self, f'setup_{self.interface_name}_problem')
        resp = self.solve_ik(setup_problem())
        if resp.success:
            # Start remapper
            if self.real_robot:
                self.toggle_remapper(True)
            self.move_to_joint_state(resp.solution, self.move_to_start_duration)
        else:
            rospy.logerr('failed to solve IK using given interface, using pybullet to solve IK')
            self.move_to_start_pose_using_pybullet()

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
