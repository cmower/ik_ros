import rospy
from sensor_msgs.msg import JointState
from ik_ros.srv import JointNames, JointNamesResponse
from custom_srvs.custom_srvs import ToggleService


class IKNode:


    def __init__(self, IKClass):

        # Initialize ROS
        rospy.init_node('ik_node')

        # Setup class variables
        self.sub = None

        # Setup ik class
        self.ik = IKClass()

        # Setup ros publisher
        self.pub = rospy.Publisher('joint_states/target', JointState, queue_size=10)

        # Setup ros services
        ToggleService('toggle_ik', self.enable, self.disable)
        rospy.Service('solve_ik', self.ik.srv_type, self.solve_ik)
        rospy.Service('joint_names', JointNames, self.joint_names)

        # Enable callback on intialization
        if rospy.get_param('~start_callback_on_init', False):
            success, message = self.enable()
            if success:
                rospy.loginfo(message)
            else:
                rospy.logerr(message)

    def joint_names(self, req):
        return JointNamesResponse(joint_names=self.ik.get_joint_names())

    def enable(self):
        if self.sub is None:
            self.sub = rospy.Subscriber('ik', self.ik.problem_msg_type, self.callback)
            message = 'registered ik callback'
            success = True
        else:
            message = 'tried to register ik callback, but it is already registered'
            success = False
        return success, message


    def disable(self):
        if self.sub is not None:
            self.sub.unregister()
            self.sub = None
            message = 'unregistered ik callback'
            success = True
        else:
            message = 'tried to unregister ik callback, but it is not registered'
            success = True
        return success, message


    def callback(self, problem):
        output = self.ik.solve(problem)
        if output.success:
            self.pub.publish(output.joint_state_msg())
        else:
            rospy.logerr('ik solver failed: %s ', output.message)

    def solve_ik(self, req):
        output = self.ik.solve(req.problem)
        return self.ik.srv_resp_type(success=output.success, message=output.message, solution=output.joint_state_msg())


    def spin(self):
        rospy.spin()
