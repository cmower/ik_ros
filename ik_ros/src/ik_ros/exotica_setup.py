import rospy
from std_msgs.msg import Float64MultiArray
from ik_ros.ik_setup_node import IKSetupNode
from ik_ros.srv import EXOTicaProblem
from ik_ros.srv import EXOTicaInfo

class EXOTicaSetupNode(IKSetupNode):

    problem_type = EXOTicaProblem

    def __init__(self):

        # Initialize
        super().__init__('exotica')

        # Get parameters
        self.max_num_prev_solutions = rospy.get_param('~max_num_prev_solutions', 1)

        # Get information on exotica
        rospy.wait_for_service('exotica_info', timeout=10.0)
        get_exotica_info = rospy.ServiceProxy('exotica_info', EXOTicaInfo)
        self.exotica_info = get_exotica_info()

        # Setup problem task map goals
        self.problem.task_map_names = self.exotica_info.task_map_names
        self.problem.task_map_goals = [
            Float64MultiArray(data=[0.0]*nrho)
            for nrho in self.exotica_info.task_map_nrho
        ]

        # Setup subscribers
        for task_map_name in self.exotica_info.task_map_names:
            topic_name = f'ik/exotica/task_map_goal/{task_map_name}'
            rospy.Subscriber(
                topic_name,
                Float64MultiArray,
                self.task_map_goal_callback,
                callback_args=task_map_name
            )

        rospy.Subscriber('previous_solutions', JointState, self.previous_solutions_callback)

        # Final intiailization
        self.post_init()

    def task_map_callback(self, msg, task_map_name):
        idx = self.problem.task_map_names.index(task_map_name)
        self.problem.task_map_goals[idx].data = msg.data

    def previous_solutions_callback(self, msg):
        self.problem.previous_solutions.append(msg)
        if len(self.problem.previous_solutions) == (self.max_num_prev_solutions+1):
            self.problem.previous_solutions.pop(0)

    def start_state_callback(self, msg):
        self.problem.start_state = msg

    def pack_problem(self):
        return True
