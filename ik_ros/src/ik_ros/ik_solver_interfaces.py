from .trac_ik.setup_node import TracIKSetupNode
from .exotica.setup_node import EXOTicaSetupNode

from .trac_ik.interface import TracIKInterface
from .exotica.interface import EXOTicaInterface

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
}
