import rospy
from sensor_msgs.msg import JointState
from dataclasses import dataclass
from typing import List
from ik_ros.msg import IKSolution

"""

This script defines a dataclass for the output of a IK interface. The
IK.solve method should output an instance of IKOutput.

"""

@dataclass
class IKOutput:
    success: bool
    message: str
    joint_names: List[str]
    solution: List[float]

    def joint_state_msg(self):
        msg = JointState(name=self.joint_names, position=self.solution)
        msg.header.stamp = rospy.Time.now()
        return msg

    def solution_msg(self, it, interface_name):
        msg = IKSolution(
            interface_name=interface_name,
            success=self.success,
            message=self.message,
            joint_names=self.joint_names,
            solution=self.solution,
        )
        msg.header.seq = it
        msg.header.stamp = rospy.Time.now()
        return msg
