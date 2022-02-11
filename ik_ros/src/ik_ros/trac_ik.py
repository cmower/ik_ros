import rospy
from std_msgs.msg import Float64MultiArray
import trac_ik_python.trac_ik as trac_ik
from ros_pybullet_interface.config import replace_package
from .ik import IK


class TracIK(IK):

    """Interface to trac_ik: https://bitbucket.org/traclabs/trac_ik.git"""


    def __init__(self):

        # Get ROS parameters
        base_link = rospy.get_param('~base_link')
        tip_link = rospy.get_param('~tip_link')
        urdf_filename = rospy.get_param('~urdf_filename')
        timeout = rospy.get_param('~timeout', 0.005)
        epsilon = rospy.get_param('~epsilon', 1e-5)
        solve_type = rospy.get_param('~solve_type', "Speed")
        self.bx=rospy.get_param('~bx', 1e-5)
        self.by=rospy.get_param('~by', 1e-5)
        self.bz=rospy.get_param('~bz', 1e-5)
        self.brx=rospy.get_param('~brx', 1e-3)
        self.bry=rospy.get_param('~bry', 1e-3)
        self.brz=rospy.get_param('~brz', 1e-3)

        # Setup variables
        self.x = None
        self.y = None
        self.z = None
        self.rx = None
        self.ry = None
        self.rz = None
        self.rw = None
        self.qinit = None
        self._solution = None
        self._did_recieve_setup = False

        # Get urdf as string
        with open(replace_package(urdf_filename), 'r') as f:
            urdf_string = f.read()

        # Setup IK solver
        self.ik_solver = trac_ik.IK(base_link, tip_link, timeout=timeout, epsilon=epsilon, solve_type=solve_type, urdf_string=urdf_string)


    def reset(self, setup):
        """Reset IK problem/solver, must be called prior to solve. Note the setup parameter must be of type std_msgs/Float64MultiArray."""
        self.x = setup.data[0]
        self.y = setup.data[1]
        self.z = setup.data[2]
        self.rx = setup.data[3]
        self.ry = setup.data[4]
        self.rz = setup.data[5]
        self.rw = setup.data[6]
        self.qinit = setup.data[7:]
        self._did_recieve_setup = True

    def did_recieve_setup(self):
        return self._did_recieve_setup


    def solve(self):
        """Calls the IK solver."""
        self._solution = self.ik_solver.get_ik(
            self.qinit,
            self.x, self.y, self.z,
            self.rx, self.ry, self.rz, self.rw,
            bx=self.bx, by=self.by, bz=self.bz,
            brx=self.brx, bry=self.bry, brz=self.brz,
        )


    def joint_names(self):
        """Return a list of joint names in same order as solution."""
        return self.ik_solver.joint_names


    def solution(self):
        """Returns the solution for the previous call to solve as a Python list."""
        return self._solution
