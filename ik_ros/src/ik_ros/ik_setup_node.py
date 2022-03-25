import rospy
from .node import RosNode
from abc import abstractmethod

class IKSetupNode(RosNode):

    def __init__(self):

        super().__init__('ik_setup')
