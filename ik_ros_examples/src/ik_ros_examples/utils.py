import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped

class FigureEightNode:

    def __init__(self):
        rospy.init_node('figure_eight_node')
        parent_frame_id = rospy.get_param('~parent_frame_id')
        child_frame_id = rospy.get_param('~child_frame_id')
        hz = rospy.get_param('~hz', 50)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        self.tf = TransformStamped()
        self.tf.header.frame_id = parent_frame_id
        self.tf.child_frame_id = child_frame_id
        self.tf.transform.rotation.w = 1.0
        rospy.Timer(rospy.Duration(1.0/float(hz)), self.main_loop)

    def main_loop(self, event):

        # Grab current time
        time_now = rospy.Time.now()
        self.tf.header.stamp = time_now

        # Update transform
        t = time_now.to_sec()
        self.tf.transform.translation.x = np.sin(t * 2.0 * np.pi * 0.5) * 0.1
        self.tf.transform.translation.y = np.sin(t * np.pi * 0.5) * 0.2

        # Broadcast transform
        self.tfBroadcaster.sendTransform(self.tf)

    def spin(self):
        rospy.spin()
