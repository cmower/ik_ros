import rospy
from .ros_node import RosNode
from sensor_msgs.msg import JointState
from ik_ros.srv import JointNames, JointNamesResponse
from ik_ros.msg import IKSolution
from custom_ros_tools.ros_comm import ToggleService

"""

This script defines the main solver node.

"""


class IKSolverNode(RosNode):


    def __init__(self, IKInterfaceClass):

        # Initialize
        super().__init__('solver', IKInterfaceClass.name)

        # Setup ik class
        self.ik = IKInterfaceClass()

        # Setup class variables
        self.it = None
        self.sub = None

        # Setup ros publisher
        self.targ_pub = rospy.Publisher('joint_states/target', JointState, queue_size=10)
        self.soln_pub = rospy.Publisher('ik/solution', IKSolution, queue_size=10)

        # Setup ros services
        rospy.Service(f'{self.ns}/solve', self.ik.srv_type, self.service_solve_ik)
        rospy.Service(f'{self.ns}/joint_names', JointNames, self.service_joint_names)

        # Post initialization
        self.post_init()

    ################################
    ## Services

    def service_solve_ik(self, req):
        output = self.ik.solve(req.problem)
        return self.ik.srv_resp_type(success=output.success, message=output.message, solution=output.joint_state_msg())

    def service_joint_names(self, req):
        return JointNamesResponse(joint_names=self.ik.get_joint_names())

    ################################
    ## Main subscriber

    def enable(self):
        if self.sub is None:
            self.it = 0
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
            self.it = None
            message = 'unregistered ik callback'
            success = True
        else:
            message = 'tried to unregister ik callback, but it is not registered'
            success = True
        return success, message


    def callback(self, problem):
        output = self.ik.solve(problem)
        if output.success:
            self.targ_pub.publish(output.joint_state_msg())
            self.soln_pub.publish(output.solution_msg(self.it, self.interface_name))
            self.it += 1
        else:
            rospy.logerr('ik solver failed: %s ', output.message)
