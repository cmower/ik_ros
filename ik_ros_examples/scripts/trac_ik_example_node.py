#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Float64MultiArray
from ik_ros_examples.utils import lwr_figure_eight
from ik_ros.srv import ToggleIK, ToggleIKRequest

def main():
    rospy.init_node('trac_ik_example_node')
    topic = 'ik_setup'
    pub = rospy.Publisher(topic)
    ik_srv = '/ik_ros/lwr_ik/toggle_ik_callback'
    rospy.wait_for_service(ik_srv)
    try:
        handle = rospy.ServiceProxy(ik_srv, ToggleIK)
        req = ToggleIKRequest(switch='on', topic=topic)
        resp = handle(req)
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: %s', str(e))
        sys.exit(0)
    rate = rospy.Rate(50)  # hz
    while not rospy.is_shutdown():
        pub.publish(Float64MultiArray(data=lwr_figure_eight(rospy.Time.now().to_sec())))
        rate.sleep()

if __name__ == '__main__':
    main()
