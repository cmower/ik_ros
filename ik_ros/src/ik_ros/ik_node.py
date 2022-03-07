import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, SetBoolResponse
from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE


class ToggleService(rospy.Service):

    def __init__(self, name, enable_handler, disable_handler,
                 buff_size=DEFAULT_BUFF_SIZE, error_handler=None):
        super(ToggleService, self).__init__(name, SetBool, self.toggle,
                                            buff_size=buff_size, error_handler=error_handler)
        self.enable_handler = enable_handler
        self.disable_handler = disable_handler

    def toggle(self, req):
        if req.data:
            success, message = self.enable_handler()
        else:
            success, message = self.disable_handler()
        return SetBoolResponse(success=success, message=message)


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
        rospy.Service('solve_ik', self.ik.get_solve_ik_srv_type(), self.solve_ik)

        # Enable callback on intialization
        if rospy.get_param('~start_callback_on_init', False):
            success, message = self.enable()
            if success:
                rospy.loginfo(message)
            else:
                rospy.logerr(message)

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
        success, message, solution = self.ik.solve(problem)
        if success:
            self.pub.publish(solution)
        else:
            rospy.logerr('ik solver failed: %s ', message)

    def solve_ik(self, req):
        success, message, solution = self.ik.solve(req.problem)
        return self.ik.srv_resp_type(success=success, message=message, solution=solution)
