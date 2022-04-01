from abc import ABCMeta, abstractmethod
from typing import List
from .ik_output import IKOutput
from sensor_msgs.msg import JointState
from custom_ros_tools.robot import resolve_joint_order

"""
This script defines the main template class for an IK interface.
"""

class IK(metaclass=ABCMeta):

    ###########################
    ## Abstract properties, must be specified in sub-class

    @property
    @abstractmethod
    def name(self) -> str:
        pass

    @property
    @abstractmethod
    def problem_msg_type(self):
        # Output: IK problem message type (defined in ik_ros/msg)
        pass

    @property
    @abstractmethod
    def srv_type(self):
        # Output: IK service type (defined in ik_ros/srv)
        pass

    @property
    @abstractmethod
    def srv_resp_type(self):
        # Output: IK service response type (defined in ik_ros/srv)
        pass

    ###########################
    ## Abstract methods, must be specified in sub-class

    @abstractmethod
    def get_joint_names(self) -> List[str]:
        pass

    @abstractmethod
    def solve(self, problem) -> IKOutput:
        # Input: IK problem (defined in ik_ros/msg)
        pass

    ###########################
    ## Other methods

    def resolve_joint_position_order(self, joint_state_msg: JointState) -> List[float]:
        return resolve_joint_order(joint_state_msg, self.get_joint_names()).position
