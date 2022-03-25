import rospy
from abc import ABC

class RosNode(ABC):

    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=True)

    def spin(self):
        rospy.spin()
