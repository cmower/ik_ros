import rospy
import pyexotica as exo
import exotica_core_task_maps_py
"""

This provides a base-class for the EXOTica interface. In order to
define the setup for each call to the solver, you need to provide a
defined reset method.


"""

class Exotica(IK):

    """Interface to Exotica: https://github.com/ipab-slmc/exotica"""

    def __init__(self):

        # Setup class attributes
        self._solution = None

        # Get ROS parameters
        xml_filename = rospy.get_param('~exotica_xml_filename')

        # Setup Exotica
        self.solver = exo.Setup.load_solver(xml_filename)
        self.problem = self.solver.get_problem()
        self.scene = self.problem.get_scene()
        self.task_maps = self.problem.get_task_maps()
        self._joint_names = self.scene.get_controlled_joint_names()

    def solve(self):
        self._solution = self.solver.solve()[0]

    def joint_names(self):
        return self._joint_names

    def solution(self):
        return self._solution
