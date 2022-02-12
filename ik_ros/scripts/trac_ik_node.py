#!/usr/bin/env python3
import rospy
from ik_ros.trac_ik import TracIK
from ik_ros.ik import IKNode

def main():
    IKNode(TracIK).spin()

if __name__ == '__main__':
    main()
