import rospy
from ..ik_setup_node import IKSetupNode
from std_msgs.msg import Float64
from ik_ros.msg import TracIKProblem
from sensor_msgs.msg import JointState

class TracIKSetupNode(IKSetupNode):

    problem_type = TracIKProblem

    def __init__(self):

        # Initialize
        super().__init__('trac_ik')

        # Get parameters
        self.parent_frame_id = rospy.get_param('~parent_frame_id')
        self.child_frame_id = rospy.get_param('~child_frame_id')

        # Setup ROS communication
        rospy.Subscriber('qinit', JointState, self.joint_state_callback)

        params = {'bx': 1e-5, 'by': 1e-5, 'bz': 1e-5, 'brx': 1e-3, 'bry': 1e-3, 'brz': 1e-3}
        for k, p in params.items():
            setattr(self.problem, k, p)
            rospy.Subscriber(k, Float64, self.param_callback, callback_args=k)

        # Final intiailization
        self.post_init()

    def joint_state_callback(self, msg):
        self.problem.qinit = msg

    def param_callback(self, msg, key):
        setattr(self.problem, key, msg.data)

    def pack_problem(self):
        tf = self.tf.get_tf_msg(self.parent_frame_id, self.child_frame_id)
        if tf:
            self.problem.goal = tf.transform
            return True
        else:
            rospy.logwarn("failed to retrieve tf, cannot setup TracIK problem!")
            return False
