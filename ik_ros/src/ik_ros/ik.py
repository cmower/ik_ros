import rospy
import tf2_ros
from std_msgs.msg import Float64MultiArray
from ik_ros.srv import ToggleIK, ToggleIKResponse
from ik_ros.srv import SolveIK, SolveIKResponse


class IK:


    def reset(self, setup):
        """Reset IK problem/solver, must be called prior to solve. Optionally provide goal in interface."""
        raise NotImplemented

    def solve(self):
        """Calls the IK solver."""
        raise NotImplemented


    def solution(self):
        """Returns the solution for the previous call to solve as a Python list."""
        raise NotImplemented


    def publish(self):
        """Publishes the IK solution to ROS."""
        raise NotImplemented


class IKNode:

    # toggle_ik_streaming: means solutions will be published at a given frequency
    # toggle_ik_callback: means solutions will be published when setup recieved
    # solve_ik: returns solution in service response

    def __init__(self, name, IKClass):
        rospy.init_node(name + '_node')
        self.name = name
        self.ik = IKClass()
        self.streaming_timer = None
        self.streaming_sub = None
        self.subscriber = None
        rospy.Service(name + '/toggle_ik_streaming', ToggleIK, self.service_toggle_ik_streaming)
        rospy.Service(name + '/toggle_ik_callback', ToggleIK, self.service_toggle_ik_callback)
        rospy.Service(name + '/solve_ik', SolveIK, self.service_solve_ik)


    def service_toggle_ik_streaming(self, req):
        info = ''
        success = True
        try:
            if req.switch:
                self.streaming_timer = rospy.Timer(rospy.Duration(1.0/float(req.hz)), self.stream)
                self.streaming_sub = rospy.Subscriber(req.topic, Float64MultiArray, self.ik.reset)
            else:
                self.streaming_timer.shutdown()
                self.streaming_sub.unregister()
                self.streaming_sub = None
                self.streaming_timer = None
        except Exception as e:
            info = "toggle_ik_streaming failed: "+str(e)
            success = False
        return ToggleIKResponse(info=info, success=success)


    def service_toggle_ik_callback(self, req):
        info = ''
        success = True
        try:
            if req.switch:
                self.subscriber = rospy.Subscriber(req.topic, Float64MultiArray, self.callback)
            else:
                self.subscriber.unregister()
                self.subscriber = None
        except Exception as e:
            info = "toggle_ik_callback failed: "+str(e)
            success = False
        return ToggleIKResponse(info=info, success=success)


    def service_solve_ik(self, req):
        info = ''
        success = True
        solution = []
        try:
            self.ik.reset(req.setup)
            self.ik.solve()
            solution = self.ik.solution()
        except Exception as e:
            info = "solve ik failed: "+str(e)
            success = False
        return SolveIKResponse(info=info, success=success, solution=solution)


    def stream(self, event):
        try:
            self.ik.solve()
            self.ik.publish()
        except Exception as e:
            rospy.logwarn('%s IK failed during streaming: '+str(e))


    def callback(self, msg):
        try:
            self.ik.reset(msg)
            self.ik.solve()
            self.ik.publish()
        except Exception as e:
            rospy.logwarn('%s IK failed in callback: '+str(e))


    def spin(self):
        rospy.spin()
