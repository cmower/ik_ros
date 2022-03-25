import rospy
from sensor_msgs.msg import JointState
from dataclasses import dataclass
from typing import List

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
