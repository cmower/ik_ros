import rospy
from custom_ros_tools.ros_comm import ToggleService
from abc import ABCMeta, abstractmethod

"""

This script defines an abstract ROS node class.

"""

class RosNode(metaclass=ABCMeta):

    def __init__(self, node_name, interface_name):
        self.node_name = node_name
        self.interface_name = interface_name
        self.ns = f'ik/{node_name}/{interface_name}'
        rospy.init_node(f'ik_{node_name}_{interface_name}', anonymous=True)
        ToggleService(f'{self.ns}/toggle', self.enable, self.disable)

    def post_init(self):
        """Must be called at the end of __init__ in sub-class."""
        if rospy.get_param('~start_on_init', False):
            success, message = self.enable()
            if success:
                rospy.loginfo(message)
            else:
                rospy.logerr(message)

    @abstractmethod
    def enable(self):
        pass

    @abstractmethod
    def disable(self):
        pass

    def spin(self):
        rospy.spin()
