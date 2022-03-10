import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
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

class FigureEightNode:

    def __init__(self):
        rospy.init_node('figure_eight_node')
        parent_frame_id = rospy.get_param('~parent_frame_id')
        child_frame_id = rospy.get_param('~child_frame_id')
        self.hz = rospy.get_param('~hz', 50)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        self.tf = TransformStamped()
        self.tf.header.frame_id = parent_frame_id
        self.tf.child_frame_id = child_frame_id
        self.tf.transform.rotation.w = 1.0
        self.timer = None
        self.start_time = None
        ToggleService('toggle_fig8', self.start, self.stop)

    def start(self):
        if self.timer is None:
            self.start_time = rospy.Time.now()
            self.timer = rospy.Timer(rospy.Duration(1.0/float(self.hz)), self.main_loop)
            success = True
            message = 'started fig8'
        else:
            success = False
            message = 'tried to start timer, but it is already running'
        return success, message

    def stop(self):
        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None
            self.start_time = None
            success = True
            message = 'stopped fig8'
        else:
            success = False
            message = 'tried to stop timer, but it is not running'
        return success, message

    def main_loop(self, event):

        # Grab current time
        time_now = rospy.Time.now() - self.start_time
        self.tf.header.stamp = rospy.Time.now()

        # Update transform
        t = time_now.to_sec()
        self.tf.transform.translation.x = np.sin(t * 2.0 * np.pi * 0.5) * 0.1
        self.tf.transform.translation.y = np.sin(t * np.pi * 0.5) * 0.2

        # Broadcast transform
        self.tfBroadcaster.sendTransform(self.tf)

    def spin(self):
        rospy.spin()
