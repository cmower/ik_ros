#!/usr/bin/env python3
import rospy
import tf2_ros
from std_msgs.msg import Float64MultiArray
from custom_srvs.custom_srvs import ToggleService

class Node:

    def __init__(self):

        # Initialize ROS
        rospy.init_node('tf_to_floatarray_node', anonymous=True)

        # Get parameters
        self.child_frame_id = rospy.get_param('~child_frame_id')
        self.parent_frame_id = rospy.get_param('~parent_frame_id')
        self.duration = rospy.Duration(1.0/float(rospy.get_param('~hz', 50)))

        # Setup ros publisher
        self.pub = rospy.Publisher('transform', Float64MultiArray, queue_size=10)

        # Set class attributes
        self.timer = None

        # Setup transform listener
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

        # Setup ROS service
        ToggleService('toggle_tf_to_floatarray', self.start, self.stop)
        if rospy.get_param('~start_on_init', False):
            self.start()

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
