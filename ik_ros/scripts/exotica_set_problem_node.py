#!/usr/bin/env python3
import rospy
from ik_ros.srv import EXOTicaProblem
from std_srvs.srv import SetBool, SetBoolResponse
from ik_ros.srv import JointNames, JointNamesResponse
from ik_ros.srv import EXOTicaInfo
from std_msgs.msg import Float64MultiArray
from custom_srvs.custom_srvs import ToggleService

class Node:

    def __init__(self):

        # Init node
        rospy.init_node('exotica_set_problem_node')
        self.exotica_problem = EXOTicaProblem()

        # Get parameters
        self.dt = 1.0/float(rospy.get_param('~hz', 50))
        self.max_num_prev_solutions = rospy.get_param('~max_num_prev_solutions', 1)

        # Get information on exotica
        rospy.wait_for_service('exotica_info')
        get_exotica_info = rospy.ServiceProxy('exotica_info', EXOTicaInfo)
        self.exotica_info = get_exotica_info()

        # Setup class variables
        self.timer = None
        self.start_state = None
        self.previous_solutions = []
        self.task_map_goals = {}

        # Setup subscribers
        for idx, task_map_name in enumerate(self.exotica_info.task_map_names):
            self.task_map_goals[task_map_name] = None
            rospy.Subscriber('exotica_set_problem/%s'%task_map_name, Float64MultiArray, self.task_map_callback, callback_args=task_map_name)

        rospy.Subscriber('previous_solutions', JointState, self.previous_solutions_callback)
        rospy.Subscriber('start_state', JointState, self.start_state_callback)

        # Final initialization
        ToggleService('toggle_exotica_set_problem_node', self.start, self.stop)
        if rospy.get_param('~start_on_init', False):
            self.start()

    def task_map_callback(self, msg, task_map_name):
        self.task_map_goals[task_map_name] = msg.data

    def previous_solutions_callback(self, msg):
        self.previous_solutions.append(msg)
        if len(self.previous_solutions) == (self.max_num_prev_solutions+1):
            self.previous_solutions.pop(0)

    def start_state_callback(self, msg):
        self.start_state = msg

    def start(self):
        if self.timer is None:
            self.timer = rospy.Timer(rospy.Duration(self.dt), self.loop)
            success = True
            message = 'started exotica_set_problem_node'
        else:
            success = False
            message = 'tried to start exotica_set_problem_node but is is already running'
        return success, message

    def stop(self):
        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None
            success = True
            message = 'stopped exotica_set_problem_node'
        else:
            success = False
            message = 'tried to stop exotica_set_problem_node but is is not running'
        return success, message

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
