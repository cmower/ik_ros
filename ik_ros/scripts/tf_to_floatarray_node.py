#!/usr/bin/env python3
import rospy
import tf2_ros
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float64MultiArray

# import rospy
# from std_srvs.srv import SetBool, SetBoolResponse
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

class Node:

    def __init__(self):
        rospy.init_node('tf_to_floatarray_node', anonymous=True)
        self.pub = rospy.Publisher('transform', Float64MultiArray, queue_size=10)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.child_frame_id = rospy.get_param('~child_frame_id')
        self.parent_frame_id = rospy.get_param('~parent_frame_id')
        hz = rospy.get_param('~hz', 50)
        self.dt = 1.0/float(hz)
        # rospy.Service('toggle_tf_to_floatarray', SetBool, self.toggle)
        self.timer = None
        ToggleService('toggle_tf_to_floatarray', self.start, self.stop)

    # def toggle(self, req):
    #     if req.data:
    #         success, message = self.start()
    #     else:
    #         success, message = self.stop()
    #     return SetBoolResponse(message=message, success=success)

    def start(self):
        if (self.timer is None):
            self.timer = rospy.Timer(rospy.Duration(self.dt), self.main_loop)
            success = True
            message = 'started tf_to_floatarray node'
            rospy.loginfo(message)
        else:
            success = False
            message = 'user attempted to start tf_to_floatarray node, but it is already running!'
            rospy.logerr(message)
        return success, message

    def stop(self):
        if (self.timer is not None):
            self.timer.shutdown()
            self.timer = None
            success = True
            message = 'stopped tf_to_floatarray node'
            rospy.loginfo(message)
        else:
            success = False
            message = 'user attempted to stop tf_to_floatarray node, but it is not running!'
            rospy.logerr(message)
        return success, message

    def main_loop(self, event):

        # Grab transform
        try:
            tf = self.tfBuffer.lookup_transform(self.parent_frame_id, self.child_frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('did not recieve transform %s in %s', self.child_frame_id, self.parent_frame_id)
            return

        # Publish transform as float array
        self.pub.publish(Float64MultiArray(data=[
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ]))

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
