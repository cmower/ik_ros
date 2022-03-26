#!/usr/bin/env python3
import sys
from ik_ros.ik_solver_interfaces import solver_interfaces

def main():
    IKSetupNodeClass = solver_interfaces[sys.argv[1]].setup_node
    IKSetupNodeClass().spin()

if __name__ == '__main__':
    main()
