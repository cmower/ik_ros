#!/usr/bin/env python3
import sys
from ik_ros.ik_solver_node import IKSolverNode
from ik_ros.ik_solver_interfaces import solver_interfaces

def main():
    IKInterfaceClass = solver_interfaces[sys.argv[1]].interface
    IKSolverNode(IKInterfaceClass).spin()

if __name__ == '__main__':
    main()
