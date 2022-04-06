
###########
## TracIK
trac_ik_available = True
try:
    from .trac_ik.setup_node import TracIKSetupNode
    from .trac_ik.interface import TracIKInterface
except ModuleNotFoundError:
    TracIKSetupNode = None
    TracIKInterface = None
    trac_ik_available = False

###########
## EXOTica
exotica_available = True
try:
    from .exotica.setup_node import EXOTicaSetupNode
    from .exotica.interface import EXOTicaInterface
except ModuleNotFoundError:
    EXOTicaSetupNode = None
    EXOTicaInterface = None
    exotica_available = False

###########
## Pybullet
## NOTE: pybullet does not have an interface (see README.md in ik_ros/src/ik_ros/pybullet)
pybullet_available = True
try:
    from .pybullet.setup_node import PybulletSetupNode
except ModuleNotFoundError:
    PybulletSetupNode = None
    pybullet_available = True

"""

This script defines a dictionary containing the IK interfaces. The
solver_interfaces dictionary is used by scripts/ik_setup_node.py and
scripts/ik_solver_node.py to load the desired interface.

"""

__all__ = ['solver_interfaces']

class SolverInterface:

    def __init__(self, setup_node, interface, is_available):
        self.setup_node = setup_node
        self.interface = interface
        self.is_available = is_available

solver_interfaces = {
    'trac_ik': SolverInterface(TracIKSetupNode, TracIKInterface, trac_ik_available),
    'exotica': SolverInterface(EXOTicaSetupNode, EXOTicaInterface, exotica_available),
    'pybullet': SolverInterface(PybulletSetupNode, None, pybullet_available),
}

for name, interface in solver_interfaces.items():
    if not interface.is_available:
        print(f">>>>>>>{name} interface is not available<<<<<<<<")
