import rospy
from std_msgs.msg import Float64MultiArray
from ..ik_setup_node import IKSetupNode
from ik_ros.msg import EXOTicaProblem
from ik_ros.srv import EXOTicaInfo
from ik_ros.msg import EXOTicaSyncTf
from sensor_msgs.msg import JointState
from rpbi.tf_interface import TfInterface

class EXOTicaSetupNode(IKSetupNode):

    problem_type = EXOTicaProblem

    def __init__(self):

        # Initialize
        super().__init__('exotica')

        # Get parameters
        self.max_num_prev_solutions = rospy.get_param('~max_num_prev_solutions', 1)
        sync_tf_to_exotica_object = rospy.get_param('~sync_tf_to_exotica_object', [])

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

        # Setup tf sync
        self.sync_tf_to_exotica_object = []
        for spec in sync_tf_to_exotica_object:
            tf_parent_frame, tf_child_frame, exo_parent_frame, exo_child_frame = spec.split(' ')
            if exo_parent_frame == "''":
                exo_parent_frame = ''
            self.sync_tf_to_exotica_object.append({'tf_parent': tf_parent_frame, 'tf_child': tf_child_frame, 'exo_parent': exo_parent_frame, 'exo_child': exo_child_frame})


        # Setup subscribers
        for task_map_name in self.exotica_info.task_map_names:
            topic_name = f'{self.ns}/task_map_goal/{task_map_name}'
            rospy.Subscriber(
                topic_name,
                Float64MultiArray,
                self.task_map_goal_callback,
                callback_args=task_map_name
            )

        rospy.Subscriber(f'{self.ns}/previous_solutions', JointState, self.previous_solutions_callback)
        rospy.Subscriber(f'{self.ns}/start_state', JointState, self.start_state_callback)

        # Final intiailization
        self.post_init()

    def task_map_goal_callback(self, msg, task_map_name):
        idx = self.problem.task_map_names.index(task_map_name)
        self.problem.task_map_goals[idx].data = msg.data

    def previous_solutions_callback(self, msg):
        self.problem.previous_solutions.append(msg)
        if len(self.problem.previous_solutions) == (self.max_num_prev_solutions+1):
            self.problem.previous_solutions.pop(0)

    def start_state_callback(self, msg):
        self.problem.start_state = msg

    def pack_problem(self):

        # Update transform frames
        sync_tf_ls = []
        for spec in self.sync_tf_to_exotica_object:
            pos, rot = self.tf.get_tf(spec['tf_parent'], spec['tf_child'])
            if pos is None: continue
            sync_tf = EXOTicaSyncTf(**spec)
            sync_tf.transform = pos+rot
            sync_tf_ls.append(sync_tf)
        self.problem.sync_tf = sync_tf_ls

        return True
