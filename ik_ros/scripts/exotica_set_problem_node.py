#!/usr/bin/env python3
import rospy
from ik_ros.srv import EXOTicaProblem
from std_srvs.srv import SetBool, SetBoolResponse
from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE
from ik_ros.srv import JointNames, JointNamesResponse
from ik_ros.srv import EXOTicaInfo
from std_msgs.msg import Float64MultiArray


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
        rospy.init_node('exotica_set_problem_node')
        self.exotica_problem = EXOTicaProblem()
        self.max_num_prev_solutions = rospy.get_param('~max_num_prev_solutions', 1)
        rospy.wait_for_service('exotica_info')
        get_exotica_info = rospy.ServiceProxy('exotica_info', EXOTicaInfo)
        self.exotica_info = get_exotica_info()
        self.dt = 1.0/float(rospy.get_param('~hz', 50))
        self.timer = None
        self.start_state = None
        self.previous_solutions = []

        self.task_map_goals = {}
        for idx, task_map_name in enumerate(self.exotica_info.task_map_names):
            self.task_map_goals[task_map_name] = None
            rospy.Subscriber('exotica_set_problem/%s'%task_map_name, Float64MultiArray, self.task_map_callback, callback_args=task_map_name)

        rospy.Subscriber('previous_solutions', JointState, self.previous_solutions_callback)
        rospy.Subscriber('start_state', JointState, self.start_state_callback)
        ToggleService('toggle_exotica_set_problem_node', self.start, self.stop)

    def task_map_callback(self, msg, task_map_name):
        self.task_map_goals[task_map_name] = msg.data

    def previous_solutions_callback(self, msg):
        self.previous_solutions.append(msg)
        if len(self.previous_solutions) == (self.max_num_prev_solutions+1):
            self.previous_solutions.pop(0)

    def start_state_callback(self, msg):
        self.start_state = msg

    def start(self):
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.loop)
        return True, 'started exotica_set_problem_node'

    def stop(self):
        self.timer.shutdown()
        self.timer = None
        return True, 'stopped exotica_set_problem_node'

    def loop(self, event):
        exotica_problem = EXOTicaProblem()
        if self.start_state is not None:
            exotica_problem.start_state = self.start_state
        if self.previous_solutions:
            exotica_problem.previous_solutions = self.previous_solutions
        for task_map_name, goal in self.task_map_goals.items():
            if goal is not None:
                exotica_problem.task_map_names.append(task_map_name)
                exotica_problem.task_map_goals.append(goal)


        exotica_problem.header.stamp = rospy.Time.now()
        self.pub.publish(exotica_problem)

    def spin(self):
        rospy.spin()

def main():
    Node().spin()

if __name__ == '__main__':
    main()
