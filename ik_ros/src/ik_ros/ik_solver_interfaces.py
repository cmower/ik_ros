from .trac_ik_interface.setup_node import TracIKSetupNode
from .exotica_interface.setup_node import EXOTicaSetupNode
from .pybullet_interface.setup_node import PybulletSetupNode
from .rbdl_interface.setup_node import RBDLSetupNode

from .trac_ik_interface.interface import TracIKInterface
from .exotica_interface.interface import EXOTicaInterface
# NOTE: pybullet does not have an interface (see README.md in ik_ros/src/ik_ros/pybullet)
from .rbdl_interface.interface import RBDLInterface

"""

This script defines a dictionary containing the IK interfaces. The
solver_interfaces dictionary is used by scripts/ik_setup_node.py and
scripts/ik_solver_node.py to load the desired interface.

"""

__all__ = ['solver_interfaces']

class SolverInterface:

    def __init__(self, setup_node, interface):
        self.setup_node = setup_node
        self.interface = interface

solver_interfaces = {
    'trac_ik': SolverInterface(TracIKSetupNode, TracIKInterface),
    'exotica': SolverInterface(EXOTicaSetupNode, EXOTicaInterface),
    'pybullet': SolverInterface(PybulletSetupNode, None),
    'rbdl': SolverInterface(RBDLSetupNode, RBDLInterface)
}
