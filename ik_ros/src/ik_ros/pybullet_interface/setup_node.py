import sys
import rospy
from sensor_msgs.msg import JointState
from ..ik_setup_node import IKSetupNode
from ros_pybullet_interface.srv import RobotInfo
from ros_pybullet_interface.msg import CalculateInverseKinematicsProblem

class PybulletSetupNode(IKSetupNode):

    problem_type = CalculateInverseKinematicsProblem

    def __init__(self):

        # Initialize
        super().__init__('pybullet')

        # Get parameters
        # TODO: consider exposing parameters as ROS subscribers
        config = rospy.get_param('~config')
        self.robot_name = config['robot_name']

        self.problem.link_name = config['link_name']
        self.problem.lowerLimits = config.get('lowerLimits', [])
        self.problem.upperLimits = config.get('upperLimits', [])
        self.problem.jointRanges = config.get('jointRanges', [])
        self.problem.resetPoses = config.get('resetPoses', [])
        self.problem.jointDamping = config.get('jointDamping', [])
        self.problem.solver = config.get('solver', 0)
        self.problem.maxNumIterations = config.get('maxNumIterations', 0)
        self.problem.residualThreshold = config.get('residualThreshold', 0.0)

        self.parent_frame_id = config['parent_frame_id']
        self.child_frame_id = config['child_frame_id']

        self.problem.dt = self.duration.to_sec()

        # Get robot info
        srv_name = f'rpbi/{self.robot_name}/robot_info'
        rospy.wait_for_service(srv_name)
        try:
            robot_info = rospy.ServiceProxy(srv_name, RobotInfo)
            self.robot_info = robot_info()
        except rospy.ServiceException as err:
            rospy.logerr(f"failed call to service: {str(err)}")
            sys.exit(0)

        # Setup subscribers
        rospy.Subscriber(f'{self.ns}/currentPosition', JointState, self.current_position_callback)

        # Final intiailization
        self.post_init()

    def current_position_callback(self, msg):
        current_position = []
        for joint in self.robot_info.joint_info:
            if joint.is_fixed: continue
            idx = msg.name.index(joint.jointName)
            current_position.append(msg.position[idx])
        self.problem.currentPosition = current_position

    def pack_problem(self):
        pos, rot = self.tf.get_tf(self.parent_frame_id, self.child_frame_id)
        if not ((pos is not None) and (rot is not None)):
            rospy.logwarn('did not recieve tf frame, cannot setup Pybullet problem')
            return False
        self.problem.targetPosition = pos
        self.problem.targetOrientation = rot
        return True
