import rospy
import tf2_ros
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import ToggleIK, ToggleIKResponse
from ik_ros.srv import SolveIK, SolveIKResponse


class IK:


    def reset(self, setup):
        """Reset IK problem/solver, must be called prior to solve. Note the setup parameter must be of type std_msgs/Float64MultiArray."""
        raise NotImplemented

    def did_recieve_setup(self):
        """Returns false when a setup has not been recieved yet, true otherwise."""
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


class IKNode:

    # toggle_ik_streaming: means solutions will be published at a given frequency
    # toggle_ik_callback: means solutions will be published when setup recieved
    # solve_ik: returns solution in service response

    def __init__(self, IKClass):
        rospy.init_node('ik_ros_node', anonymous=True)
        self.ik = IKClass()
        self.streaming_timer = None
        self.streaming_sub = None
        self.subscriber = None
        self.publisher = rospy.Publisher('joint_states/target', JointState, queue_size=10)
        name = rospy.get_param('~ik_name')
        rospy.Service('/ik_ros/%s/toggle_ik_streaming' % name, ToggleIK, self.service_toggle_ik_streaming)
        rospy.Service('/ik_ros/%s/toggle_ik_callback' % name, ToggleIK, self.service_toggle_ik_callback)
        rospy.Service('/ik_ros/%s/solve_ik' % name, SolveIK, self.service_solve_ik)


    def service_toggle_ik_streaming(self, req):
        message = ''
        success = True
        try:
            if req.switch:
                self.streaming_timer = rospy.Timer(rospy.Duration(1.0/float(req.hz)), self.stream)
                self.streaming_sub = rospy.Subscriber(req.topic, Float64MultiArray, self.ik.reset)
                rospy.loginfo('turned on ik streaming')
            else:
                self.streaming_timer.shutdown()
                self.streaming_sub.unregister()
                self.streaming_sub = None
                self.streaming_timer = None
                rospy.loginfo('turned off ik streaming')
        except Exception as e:
            message = "toggle_ik_streaming failed: "+str(e)
            success = False
        return ToggleIKResponse(message=message, success=success)


    def service_toggle_ik_callback(self, req):
        message = ''
        success = True
        try:
            if req.switch:
                self.subscriber = rospy.Subscriber(req.topic, Float64MultiArray, self.callback)
                rospy.loginfo('turned on ik callback')
            else:
                self.subscriber.unregister()
                self.subscriber = None
                rospy.loginfo('turned off ik callback')
        except Exception as e:
            message = "toggle_ik_callback failed: "+str(e)
            success = False
        return ToggleIKResponse(message=message, success=success)


    def service_solve_ik(self, req):
        message = ''
        success = True
        solution = []
        try:
            self.ik.reset(req.setup)
            self.ik.solve()
            solution = self.ik.solution()
        except Exception as e:
            message = "solve ik failed: "+str(e)
            success = False
        return SolveIKResponse(message=message, success=success, solution=solution)


    def publish(self, n, q):
        msg = JointState(name=n, position=q)
        msg.header.stamp = rospy.Time.now()
        self.publisher.publish(msg)


    def stream(self, event):
        if not self.ik.did_recieve_setup():
            return
        try:
            self.ik.solve()
            self.publish(self.ik.joint_names(), self.ik.solution())
        except Exception as e:
            rospy.logwarn('%s IK failed during streaming: '+str(e))


    def callback(self, msg):
        try:
            self.ik.reset(msg)
            self.ik.solve()
            self.publish(self.ik.joint_names(), self.ik.solution())
        except Exception as e:
            rospy.logwarn('%s IK failed in callback: '+str(e))


    def spin(self):
        rospy.spin()
