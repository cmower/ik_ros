import rospy
from std_msgs.msg import Float64MultiArray
import trac_ik_python.trac_ik as trac_ik
from ros_pybullet_interface.config import replace_package
from .ik import IK


class TracIK(IK):


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

        # Get urdf as string
        with open(replace_package(urdf_filename), 'r') as f:
            urdf_string = f.read()

        # Setup IK solver
        self.ik_solver = trac_ik.IK(base_link, tip_link, timeout=timeout, epsilon=epsilon, solve_type=solve_type, urdf_string=urdf_string)

        # Setup publisher
        self.pub = rospy.Publisher('ik/trac_ik/joint_state', Float64MultiArray, queue_size=10)


    def reset(self, setup):
        """Reset IK problem/solver. Optionally provide goal in interface."""
        self.x = setup[0]
        self.y = setup[1]
        self.z = setup[2]
        self.rx = setup[3]
        self.ry = setup[4]
        self.rz = setup[5]
        self.rw = setup[6]
        self.qinit = setup[7:]


    def solve(self):
        """Calls the IK solver."""
        self._solution = self.ik_solver.get_ik(
            self.qinit,
            self.x, self.y, self.z,
            self.rx, self.ry, self.rz, self.rw,
            bx=self.bx, by=self.by, bz=self.bz,
            brx=self.brx, bry=self.bry, brz=self.brz,
        )


    def solution(self):
        """Returns the solution for the previous call to solve as a Python list."""
        return self._solution


    def publish(self):
        """Publishes the IK solution to ROS."""
        self.pub.publish(Float64MultiArray(data=self._solution))
