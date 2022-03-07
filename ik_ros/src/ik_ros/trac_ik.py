import rospy
from .ik_interface import IK
from sensor_msgs.msg import JointState
from ik_ros.msg import TracIKProblem
from ik_ros.srv import TracIK, TracIKResponse
from rpbi.config import replace_package
from trac_ik_python.trac_ik_wrap import TRAC_IK

class TracIKInterface(IK):

    """Interface to trac_ik: https://bitbucket.org/traclabs/trac_ik.git"""

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
        self.bx = rospy.get_param('~bx', 1e-5)
        self.by = rospy.get_param('~by', 1e-5)
        self.bz = rospy.get_param('~bz', 1e-5)
        self.brx = rospy.get_param('~brx', 1e-3)
        self.bry = rospy.get_param('~bry', 1e-3)
        self.brz = rospy.get_param('~brz', 1e-3)

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
            self.bx, self.by, self.bz, self.brx, self.bry, self.brz,
        )

        # Pack solution
        if solution:
            success = True
            message = 'trac_ik succeeded'
            solution_ = JointState(name=self.get_joint_names(), position=solution)
        else:
            success = False
            message = 'trac_ik failed'
            solution_ = JointState(name=self.get_joint_names())
        solution_.header.stamp = rospy.Time.now()

        return success, message, solution_
