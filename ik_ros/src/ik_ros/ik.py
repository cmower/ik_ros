import rospy
import tf2_ros
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from ik_ros.srv import ToggleIK, ToggleIKResponse
from ik_ros.srv import SolveIK, SolveIKResponse


class IK:


    def reset(self, setup):
        """Reset IK problem/solver, must be called prior to solve. Note the setup parameter must be of type std_msgs/Float64MultiArray."""
        raise NotImplemented

    def solve(self):
        """Calls the IK solver."""
        raise NotImplemented

    def joint_names(self):
        """Return a list of joint names in same order as solution."""
        raise NotImplemented

    def solution(self):
        """Returns the solution for the previous call to solve as a Python list."""
        raise NotImplemented


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


class SetupSubscriber:


    def __init__(self, callback_handle):
        self.sub = None
        self.is_running = False
        self.did_recieve_setup = None
        self.callback_handle = callback_handle


    def start(self):
        self.did_recieve_setup = False
        self.sub = rospy.Subscriber('setup', Float64MultiArray, self.callback)
        self.is_running = True


    def callback(self, msg):
        self.did_recieve_setup = True
        self.callback_handle(msg)


    def stop(self):
        self.sub.unregister()
        self.is_running = False
        self.did_recieve_setup = None


class IKNode:

    # toggle_ik_streaming: means solutions will be published at a given frequency
    # toggle_ik_callback: means solutions will be published when setup recieved
    # solve_ik: returns solution in service response


    def __init__(self, IKClass):

        # Initialize ROS node
        rospy.init_node('ik_ros_node', anonymous=True)

        # Initialize IK interface
        self.ik = IKClass()

        # Setup joint state publisher
        self.joint_state_publisher = JointStatePublisher(self.ik.joint_names())

        # Setup setup subscribers
        self.streaming_subscriber = SetupSubscriber(self.ik.reset)
        self.callback_subscriber = SetupSubscriber(self.callback)

        # Setup ROS services
        rospy.Service('/toggle_ik_streaming', ToggleIK, self.service_toggle_ik_streaming)
        rospy.Service('/toggle_ik_callback', ToggleIK, self.service_toggle_ik_callback)
        rospy.Service('/solve_ik', SolveIK, self.service_solve_ik)


    def spin(self):
        rospy.spin()


    ##############################################
    ## Streaming


    def service_toggle_ik_streaming(self, req):
        if req.switch:
            success, message = self.enable_ik_streaming(req)
        else:
            success, message = self.disable_ik_streaming(req)
        return ToggleIKResponse(message=message, success=success)


    def enable_ik_streaming(self, req):
        success = True
        message = 'enabled ik streaming'
        if not self.streaming_subscriber.is_running:

            if not self.callback_subscriber.is_running:
                # Turn on IK streaming
                self.streaming_subscriber.start()
                self.streaming_timer = rospy.Timer(rospy.Duration(1.0/float(req.hz)), self.stream)
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


    def disable_ik_streaming(self, req):
        success = True
        message = ''
        if self.streaming_subscriber.is_running:
            self.streaming_timer.shutdown()
            self.streaming_subscriber.stop()
            self.streaming_timer = None
            rospy.loginfo('turned off ik streaming')
        else:
            success = False
            message = 'user attempted to turn off ik streaming, but it is not running!'
            rospy.logerr(message)
        return success, message


    def stream(self, event):
        if not self.streaming_subscriber.did_recieve_setup:
            return
        try:
            self.ik.solve()
            self.joint_state_publisher.publish(self.ik.solution())
        except Exception as e:
            rospy.logwarn('IK failed during streaming: '+str(e))


    ##############################################
    ## Callback


    def service_toggle_ik_callback(self, req):
        if req.switch:
            success, message = self.enable_ik_callback(req)
        else:
            success, message = self.disable_ik_callback(req)
        return ToggleIKResponse(message=message, success=success)


    def enable_ik_callback(self, req):
        success = True
        message = 'enabled ik callback'
        if not self.callback_subscriber.is_running:

            if not self.streaming_subscriber.is_running:
                self.callback_subscriber.start()
            else:
                success = False
                message = 'user attempted to turn on ik callback, but ik streaming is already running!'
                rospy.logerr(message)
        else:
            success = False
            message = 'user attempted to turn on ik callback, but it  is already running!'
            rospy.logerr(message)

        return success, message


    def callback(self, msg):
        try:
            self.ik.reset(msg)
            self.ik.solve()
            self.joint_state_publisher.publish(self.ik.solution())
        except Exception as e:
            rospy.logwarn('IK failed in callback: '+str(e))


    ##############################################
    ## Solve IK service


    def service_solve_ik(self, req):

        # Check streaming/callback is not enabled
        if self.streaming_subscriber.is_running:
            success = False
            solution = []
            message = 'cannot solve IK when streaming ik is enabled'
            return SolveIKResponse(message=message, success=success, solution=solution)

        if self.callback_subscriber.is_running:
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
