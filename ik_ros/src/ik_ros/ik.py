import rospy
import tf2_ros
from functools import partial
from abc import ABC, abstractmethod
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, SetBoolResponse
from ik_ros.srv import SolveIK, SolveIKResponse
from ik_ros.srv import JointNameOrder, JointNameOrderResponse


class IK(ABC):


    @abstractmethod
    def reset(self, setup):
        pass

    @abstractmethod
    def solve(self):
        pass

    @abstractmethod
    def joint_names(self):
        pass

    @abstractmethod
    def solution(self):
        pass


class JointStatePublisher:

    def __init__(self, joint_names):
        self.pub = rospy.Publisher('joint_states/target', JointState, queue_size=10)
        self.msg = JointState(name=joint_names)

    def pack_message(self, position):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.position = position
        return self.msg

    def publish(self, position):
        self.pub.publish(self.pack_message(position))


class IkCallback:


    def __init__(self, callback_handle):
        self.sub = None
        self.did_recieve_setup = None
        self.callback_handle = callback_handle

    @property
    def is_running(self):
        return self.sub is not None

    def start(self):
        self.did_recieve_setup = False
        self.sub = rospy.Subscriber('setup', Float64MultiArray, self.callback)


    def callback(self, msg):
        self.did_recieve_setup = True
        self.callback_handle(msg)


    def stop(self):
        self.sub.unregister()
        self.sub = None
        self.did_recieve_setup = None

class IKNode:

    # toggle_ik_streaming: means solutions will be published at a given frequency
    # toggle_ik_callback: means solutions will be published when setup recieved
    # solve_ik: returns solution in service response


    def __init__(self, IKClass):

        # Initialize ROS node
        rospy.init_node('ik_ros_node', anonymous=True)

        # Get ros parameters
        hz = rospy.get_param('~hz', 100)
        self.dt = 1.0/float(hz)

        # Initialize IK interface
        self.ik = IKClass()

        # Setup joint state publisher
        self.joint_state_publisher = JointStatePublisher(self.ik.joint_names())

        # Setup setup subscribers
        self.streaming_ik_callback = IkCallback(self.ik.reset)
        self.callback_ik_callback = IkCallback(self.callback)

        # Setup ROS services
        # NOTE: if multiple IK nodes are used then service names will need to be remapped
        rospy.Service('/toggle_ik_streaming', SetBool, partial(self.toggle, enable_handle=self.enable_ik_streaming, disable_handle=self.disable_ik_streaming))
        rospy.Service('/toggle_ik_callback', SetBool, partial(self.toggle, enable_handle=self.enable_ik_callback, disable_handle=self.disable_ik_callback))
        rospy.Service('/solve_ik', SolveIK, self.service_solve_ik)
        rospy.Service('/joint_name_order', JointNameOrder, self.service_joint_name_order)

        # Start on init
        if rospy.get_param('~start_callback_on_init', False):
            self.enable_ik_callback()


    def spin(self):
        rospy.spin()


    def toggle(self, req, enable_handle, disable_handle):
        if req.data:
            success, message = enable_handle()
        else:
            success, message = disable_handle()
        return SetBoolResponse(success=success, message=message)


    def service_joint_name_order(self):
        return JointNameOrderResponse(joint_name=self.ik.joint_names())


    def enable_ik_streaming(self):
        success = True
        message = 'enabled ik streaming'
        if not self.streaming_ik_callback.is_running:

            if not self.callback_ik_callback.is_running:
                # Turn on IK streaming
                self.streaming_ik_callback.start()
                self.streaming_timer = rospy.Timer(rospy.Duration(self.dt), self.stream)
                rospy.loginfo('enabled ik streaming')

            else:
                # IK callback is running
                success = False
                message = 'user attempted to enable ik streaming, but the callback is already running!'
                rospy.logerr(message)

        else:
            success = False
            message = 'user attempted to turn on ik streaming, but it is already running!'
            rospy.logerr(message)

        return success, message


    def disable_ik_streaming(self):
        success = True
        message = ''
        if self.streaming_ik_callback.is_running:
            self.streaming_timer.shutdown()
            self.streaming_ik_callback.stop()
            self.streaming_timer = None
            rospy.loginfo('turned off ik streaming')
        else:
            success = False
            message = 'user attempted to turn off ik streaming, but it is not running!'
            rospy.logerr(message)
        return success, message


    def stream(self, event):
        if not self.streaming_ik_callback.did_recieve_setup:
            return
        try:
            self.ik.solve()
            self.joint_state_publisher.publish(self.ik.solution())
        except Exception as e:
            rospy.logwarn('IK failed during streaming: '+str(e))

    def enable_ik_callback(self):
        success = True
        message = 'enabled ik callback'
        if not self.callback_ik_callback.is_running:

            if not self.streaming_ik_callback.is_running:
                self.callback_ik_callback.start()
            else:
                success = False
                message = 'user attempted to turn on ik callback, but ik streaming is already running!'
                rospy.logerr(message)
        else:
            success = False
            message = 'user attempted to turn on ik callback, but it  is already running!'
            rospy.logerr(message)

        return success, message

    def disable_ik_callback(self):
        success = True
        message = 'disabled ik callback'
        if self.callback_ik_callback.is_running:
            self.callback_ik_callback.stop()
        else:
            success = False
            message = 'user attempted to turn off ik callback, but it is not running!'
            rospy.logerr(message)
        return success, message


    def callback(self, msg):
        try:
            self.ik.reset(msg)
            self.ik.solve()
            self.joint_state_publisher.publish(self.ik.solution())
        except Exception as e:
            rospy.logwarn('IK failed in callback: '+str(e))

    def service_solve_ik(self, req):

        # Check streaming/callback is not enabled
        if self.streaming_ik_callback.is_running:
            success = False
            solution = []
            message = 'cannot solve IK when streaming ik is enabled'
            return SolveIKResponse(message=message, success=success, solution=solution)

        if self.callback_ik_callback.is_running:
            success = False
            solution = []
            message = 'cannot solve IK when callback ik is enabled'
            return SolveIKResponse(message=message, success=success, solution=solution)

        # Solve IK
        try:
            self.ik.reset(req.setup)
            self.ik.solve()
            message = 'solved ik'
            success = True
            solution = self.joint_state_publisher.pack_message(self.ik.solution())
        except Exception as e:
            message = "solve ik failed: "+str(e)
            success = False

        return SolveIKResponse(message=message, success=success, solution=solution)
