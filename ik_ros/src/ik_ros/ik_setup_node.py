import rospy
from custom_ros_tools.tf import TfInterface
from .ros_node import RosNode
from abc import abstractmethod

"""

This script defines the template class for a ROS node that collects
parameters and sends it to the solver node.

"""

class IKSetupNode(RosNode):

    def __init__(self, interface_name):

        # Setup ros
        super().__init__('setup', interface_name)

        # Setup tf interface
        self.tf = TfInterface()

        # Setup class attributes
        self.timer = None

        # Setup problem and publisher
        self.pub = rospy.Publisher('ik', self.problem_type, queue_size=10)
        self.problem = self.problem_type()

        # Setup class attributes
        self.duration = rospy.Duration(1.0/float(rospy.get_param('~hz', 50))) # node sampling freq

    @property
    @abstractmethod
    def problem_type(self):
        # Output: IK problem type (defined in ik_ros/msg)
        pass

    @abstractmethod
    def pack_problem(self) -> bool:
        """Packs the problem, returns true if the problem is ok to publish"""
        pass

    def enable(self):
        if self.timer is None:
            self.timer = rospy.Timer(self.duration, self.main_loop)
            success = True
            message = f"started ik/{self.node_name}/{self.interface_name}"
        else:
            success = False
            message = f'tried to start ik/{self.node_name}/{self.interface_name} but it is already running'
        return success, message

    def disable(self):
        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None
            success = True
            message = f"stopped ik/{self.node_name}/{self.interface_name}"
        else:
            success = False
            message = f'tried to stop ik/{self.node_name}/{self.interface_name} but it is not running'
        return success, message

    def main_loop(self, event):
        if self.pack_problem():
            self.problem.header.stamp = rospy.Time.now()
            self.pub.publish(self.problem)
