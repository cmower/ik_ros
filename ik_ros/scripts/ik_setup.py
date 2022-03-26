#!/usr/bin/env python3
import sys
from ik_ros.trac_ik_setup import TracIKSetupNode

def main():

    # Get ik interface
    ik_class_str = sys.argv[1]
    try:
        IKSetupNodeClass = eval(ik_class_str)
    except NameError:
        raise ValueError("Did not recognize argument!")

    # Start node and spin
    IKSetupNodeClass().spin()

if __name__ == '__main__':
    main()
