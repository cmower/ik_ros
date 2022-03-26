#!/usr/bin/env python3
import sys

# Import main ik node
from ik_ros.ik_solver_node import IKSolverNode

# Import all interface classes
from ik_ros.trac_ik import TracIKInterface
from ik_ros.exotica import EXOTicaInterface

def main():

    # Get ik interface
    ik_class_str = sys.argv[1]
    try:
        IKInterfaceClass = eval(ik_class_str)
    except NameError:
        raise ValueError("Did not recognize argument!")

    # Start node and spin
    IKSolverNode(IKInterfaceClass).spin()

if __name__ == '__main__':
    main()
