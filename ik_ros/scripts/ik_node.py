#!/usr/bin/env python3
import sys

# Import main ik node
from ik_ros.ik_node import IKNode

# Import all interface classes
from ik_ros.trac_ik import TracIKInterface
from ik_ros.exotica import ExoticaInterface

def main():

    # Get ik interface
    ik_class_str = sys.argv[1]
    try:
        IKClass = eval(ik_class_str)
    except NameError:
        raise ValueError("Did not recognize user input! In the command line input you must give the class name.")

    # Start node and spin
    IKNode(IKClass).spin()

if __name__ == '__main__':
    main()
