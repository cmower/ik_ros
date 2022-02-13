#!/usr/bin/env python3
import rospy
import tf2_ros
from std_msgs.msg import Float64MultiArray


def main():

    # Setup
    rospy.init_node('tf_to_floatarray_node', anonymous=True)
    pub = rospy.Publisher('transform', Float64MultiArray, queue_size=10)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    child_frame_id = rospy.get_param('~child_frame_id')
    parent_frame_id = rospy.get_param('~parent_frame_id')
    hz = rospy.get_param('~hz', 50)
    rate = rospy.Rate(hz)

    # Main loop
    while not rospy.is_shutdown():

        # Grab transform
        try:
            tf = tfBuffer.lookup_transform(parent_frame_id, child_frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('did not recieve transform %s in %s', child_frame_id, parent_frame_id)
            rate.sleep()
            continue

        # Publish transform as float array
        pub.publish(Float64MultiArray(data=[
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ]))

        # Sleep
        rate.sleep()


if __name__ == '__main__':
    main()
