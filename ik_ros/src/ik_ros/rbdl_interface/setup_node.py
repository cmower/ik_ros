import rospy
from sensor_msgs.msg import JointState
from ..ik_setup_node import IKSetupNode
from ik_ros.msg import RBDLProblem

"""

Original code by Theodoros Stouraitis (@stoutheo). Adapted by Chris E. Mower for the ik_ros package (@cmower).

"""


class RBDLSetupNode(IKSetupNode):

    problem_type = RBDLProblem

    def __init__(self):

        # Initialize
        super().__init__('rbdl')

        # Get parameters
        self.parent_frame_id = rospy.get_param('~parent_frame_id')
        self.child_frame_id = rospy.get_param('~child_frame_id')

        # Setup subscriber
        rospy.Subscriber(f'{self.ns}/current_position', JointState, self.callback)

        self.post_init()

    def callback(self, msg):
        self.problem.current_position = msg

    def pack_problem(self):
        tf = self.tf.get_tf_msg(self.parent_frame_id, self.child_frame_id)
        if tf is not None:
            self.problem.target_EE_transform = tf.transform
            return True
        else:
            rospy.logwarn("failed to retrieve tf, cannot setup RBDL problem!")
            return False
