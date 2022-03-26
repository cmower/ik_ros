import rospy
import numpy as np
from .ik_interface import IK
from .ik_output import IKOutput
from ik_ros.msg import TracIKProblem
from ik_ros.srv import TracIK, TracIKResponse
from rpbi.config import replace_package
from trac_ik_python.trac_ik_wrap import TRAC_IK

class TracIKInterface(IK):

    """Interface to trac_ik: https://bitbucket.org/traclabs/trac_ik.git"""

    name = 'trac_ik'
    problem_msg_type = TracIKProblem
    srv_type = TracIK
    srv_resp_type = TracIKResponse

    def __init__(self):

        # Get ROS parameters
        urdf_filename = rospy.get_param('~urdf_filename')
        base_link = rospy.get_param('~base_link')
        tip_link = rospy.get_param('~tip_link')
        timeout = np.clip(rospy.get_param('~timeout', 0.005), 0.0, np.inf)
        epsilon = np.clip(rospy.get_param('~epsilon', 1e-5), 0.0, np.inf)
        solve_type = rospy.get_param('~solve_type', "Speed")

        # Load solver and get joint names
        with open(replace_package(urdf_filename), 'r') as urdf:
            urdf_string = urdf.read()
            self.ik_solver = TRAC_IK(base_link, tip_link, urdf_string, timeout, epsilon, solve_type)
            self._joint_names = self.ik_solver.getJointNamesInChain(urdf_string)

        rospy.loginfo('initialized TracIK server')


    def get_joint_names(self):
        return self._joint_names


    def solve(self, problem):

        # Solve ik problem
        solution = self.ik_solver.CartToJnt(
            self.resolve_joint_position_order(problem.qinit),
            problem.goal.translation.x, problem.goal.translation.y, problem.goal.translation.z,
            problem.goal.rotation.x, problem.goal.rotation.y, problem.goal.rotation.z, problem.goal.rotation.w,
            problem.bx, problem.by, problem.bz, problem.brx, problem.bry, problem.brz,
        )

        # Pack IK output
        if solution:
            success = True
            message = 'trac_ik succeeded'
        else:
            success = False
            message = 'trac_ik failed'
            solution = []  # before solution is None

        return IKOutput(success, message, self.get_joint_names(), solution)
