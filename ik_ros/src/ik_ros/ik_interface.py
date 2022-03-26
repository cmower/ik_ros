from abc import ABCMeta, abstractmethod

"""
This script defines the main template class for an IK interface.
"""

class IK(metaclass=ABCMeta):

    ###########################
    ## Abstract properties, must be specified in sub-class

    @property
    @abstractmethod
    def name(self):
        pass

    @property
    @abstractmethod
    def problem_msg_type(self):
        pass

    @property
    @abstractmethod
    def srv_type(self):
        pass

    @property
    @abstractmethod
    def srv_resp_type(self):
        pass

    ###########################
    ## Abstract methods, must be specified in sub-class

    @abstractmethod
    def get_joint_names(self):
        pass

    @abstractmethod
    def solve(self, problem):
        pass

    ###########################
    ## Other methods

    def resolve_joint_position_order(self, joint_state_msg):
        position = []
        for name in self.get_joint_names():
            if name in joint_state_msg.name:
                idx = joint_state_msg.name.index(name)
                position.append(joint_state_msg.position[idx])
            else:
                # Position not given -> fill with 0.0
                position.append(0.0)
        return position
