#!/usr/bin/env python3
import rospy
import tf2_ros
import tf_conversions
from std_msgs.msg import Float64MultiArray

class Node:

    def __init__(self):

        # Initialize ROS
        rospy.init_node('tf_to_floatarray_node', anonymous=True)

        # Get parameters
        self.child_frame_id = rospy.get_param('~child_frame_id')
        self.parent_frame_id = rospy.get_param('~parent_frame_id')
        self.duration = rospy.Duration(1.0/float(rospy.get_param('~hz', 50)))
        self.rot_as_eul = rospy.get_param('~rot_as_eul', False)

        # Setup publish method
        if self.rot_as_eul:
            self.publish = self.publish_tf_as_pos_eul
        else:
            self.publish = self.publish_tf_as_pos_quat

        # Setup ros publisher
        self.pub = rospy.Publisher('transform', Float64MultiArray, queue_size=10)

        # Setup transform listener
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

        # Start main loop
        rospy.Timer(rospy.Duration(self.dt), self.main_loop)

    def publish_tf_as_pos_quat(self, tf):
        self.pub.publish(Float64MultiArray(data=[
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ]))

    def publish_tf_as_pos_eul(self, tf):
        eul = tf_conversions.transformations.euler_from_quaternion([getattr(tf.transform.rotation, dim) for dim in 'xyzw'])
        self.pub.publish(Float64MultiArray(data=[
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
            eul[0],
            eul[1],
            eul[2],
        ]))

    def main_loop(self, event):

        # Grab transform
        try:
            tf = self.tfBuffer.lookup_transform(self.parent_frame_id, self.child_frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('did not recieve transform %s in %s', self.child_frame_id, self.parent_frame_id)
            return

        # Publish transform as float array
        # NOTE: self.publish method is set in init, it is either
        # publish_tf_as_pos_quat (default) or publish_tf_as_pos_eul -
        # this is determined by the value of the ROS parameter
        # '~rot_as_eul'.
        self.publish(tf)

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
