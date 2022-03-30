#!/usr/bin/env python3
import rospy
import tf_conversions
from custom_ros_tools.tf import TfInterface
from std_msgs.msg import Float64MultiArray

class Node:

    def __init__(self):

        # Initialize ROS
        rospy.init_node('tf_to_floatarray_node', anonymous=True)

        # Get parameters
        self.child_frame_id = rospy.get_param('~child_frame_id')
        self.parent_frame_id = rospy.get_param('~parent_frame_id')
        self.duration = rospy.Duration(1.0/float(abs(rospy.get_param('~hz', 50))))
        self.mode = rospy.get_param('~mode', 'pos+quat')

        # Setup publish method
        if self.mode == 'pos+eul':
            self.publish = self.publish_tf_as_pos_eul
        elif self.mode == 'pos+quat':
            self.publish = self.publish_tf_as_pos_quat
        elif self.mode == 'pos':
            self.publish = self.publish_tf_as_pos
        else:
            raise ValueError("did not recognize mode! use either: 'pos', 'pos+eul', 'pos+quat'")

        # Setup ros publisher
        self.pub = rospy.Publisher('transform', Float64MultiArray, queue_size=10)

        # Setup tf interface
        self.tf = TfInterface()

        # Start main loop
        rospy.Timer(self.duration, self.main_loop)

    def publish_tf_as_pos(self, tf):
        self.pub.publish(Float64MultiArray(data=self.tf.msg_to_pos(tf)))

    def publish_tf_as_pos_quat(self, tf):
        p, q = self.tf.msg_to_pos_quat(tf)
        self.pub.publish(Float64MultiArray(data=p.tolist()+q.tolist()))

    def publish_tf_as_pos_eul(self, tf):
        p, e = self.tf.msg_to_pos_eul(tf)
        self.pub.publish(Float64MultiArray(data=p.tolist()+e.tolist())

    def main_loop(self, event):

        # Grab transform
        try:
            tf = self.tfBuffer.lookup_transform(self.parent_frame_id, self.child_frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('did not recieve transform %s in %s', self.child_frame_id, self.parent_frame_id)
            return

        # Publish transform as float array
        # NOTE: self.publish method is set in init, it is determined
        # by the ROS parameter '~mode'.
        self.publish(tf)

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
