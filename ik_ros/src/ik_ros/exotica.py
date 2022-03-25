import rospy
from .ik_interface import IK
from .ik_output import IKOutput
from sensor_msgs.msg import JointState
import pyexotica as exo
import exotica_core_task_maps_py
import exotica_scipy_solver
from ik_ros.msg import EXOTicaProblem
from ik_ros.srv import EXOTica, EXOTicaResponse
from ik_ros.srv import EXOTicaInfo, EXOTicaInfoResponse
from rpbi.tf_interface import TfInterface


class EXOTicaInterface(IK):

    """Interface to Exotica: https://github.com/ipab-slmc/exotica"""

    problem_msg_type = EXOTicaProblem
    srv_type = EXOTica
    srv_resp_type = EXOTicaResponse

    joint_smoothing_task_map_types = (
        exotica_core_task_maps_py.JointVelocityBackwardDifference,
        exotica_core_task_maps_py.JointAccelerationBackwardDifference,
        exotica_core_task_maps_py.JointJerkBackwardDifference,
    )

    def __init__(self):

        ##############################
        ## Initialize exotica info

        self.exotica_info = EXOTicaResponse()

        ##############################
        ## Get ROS parameters
        xml_filename = rospy.get_param('~xml_filename')
        self.exotica_info.xml_filename = xml_filename

        # Scipy solver
        use_scipy_solver = rospy.get_param('~use_scipy_solver', False)
        scipy_solver_method = rospy.get_param('~scipy_solver_method', 'SLSQP')
        self.exotica_info.use_scipy_solver = use_scipy_solver
        self.exotica_info.scipy_solver_method = scipy_solver_method

        # Sync tf to object transform
        sync_tf_to_exotica_object = rospy.get_param('~sync_tf_to_exotica_object', [])
        self.exotica_info.sync_tf_to_exotica_object = sync_tf_to_exotica_object

        ##############################
        ## Setup tf interface

        self.tf = TfInterface()

        ##############################
        ## Load EXOTica

        # Load problem and solver
        solver = None
        try:
            solver = exo.Setup.load_solver(xml_filename)
        except RunTimeError as e:
            rospy.loginfo('no solver specified in EXOTica XML, defaulting to Scipy solver')
            use_scipy_solver = True

        if not use_scipy_solver:
            self.solver = solver
            self.problem = self.solver.get_problem()
        else:
            self.problem = exo.Setup.load_problem(xml_filename)
            self.solver = exotica_scipy_solver.SciPyEndPoseSolver(problem=self.problem, method=scipy_solver_method, debug=False)

        # Load scene and task maps
        self.scene = self.problem.get_scene()
        self.task_maps = self.problem.get_task_maps()
        for name, task_map in self.task_maps.items():
            self.exotica_info.task_map_names.append(name)
            self.exotica_info.task_map_types.append(task_map.__name__)

        # Get joint names
        self._joint_names = self.scene.get_controlled_joint_names()
        self.exotica_info.controlled_joint_names = self._joint_names

        # Setup joint smoothing task maps
        self.joint_smoothing_task_maps = [task_map for task_map in self.task_maps.values() if isinstance(task_map, self.joint_smoothing_task_map_types)]

        ##############################
        ## Setup tf syncs

        self.sync_tf_to_exotica_object = []
        for spec in sync_tf_to_exotica_object:
            tf_parent_frame, tf_child_frame, exo_parent_frame, exo_child_frame = spec.split(' ')
            if exo_parent_frame == "''":
                exo_parent_frame = ''
            self.sync_tf_to_exotica_object.append({'tf_parent': tf_parent_frame, 'tf_child': tf_child_frame, 'exo_parent': exo_parent_frame, 'exo_child': exo_child_frame})

        ##############################
        ## Setup services

        rospy.Service('exotica_info', EXOTicaInfo, self.service_exotica_info)

    def get_joint_names(self):
        return self._joint_names

    def solve(self, problem):

        ##############################
        ## Setup problem

        # Update transform frames
        for spec in self.sync_tf_to_exotica_object:
            pos, rot = self.tf.get_tf(spec['tf_parent'], spec['tf_child'])
            if pos is None: continue
            self.scene.attach_object_local(spec['exo_child'], spec['exo_parent'], pos+rot)

        # Update problem start state
        if problem.start_state.position:
            self.problem.start_state = self.resolve_joint_position_order(problem.start_state)

        # Update task maps
        for name, goal in zip(problem.task_map_names, problem.task_map_goals):
            self.task_maps[name].set_goal(goal)

        # Update joint smoothing task maps
        if problem.previous_solutions and self.joint_smoothing_task_maps:
            for previous_solution in problem.previous_solutions:
                previous_joint_state = self.resolve_joint_position_order(previous_solution)
                for task_map in self.joint_smoothing_task_maps:
                    task_map.set_previous_joint_state(previous_joint_state)

        ##############################
        ## Solve problem

        try:
            solution = self.solver.solve()[0].tolist()
            success = True
            message = 'exotica ik succeeded'
        except:
            solution = []
            success = False
            message = 'exotica ik failed'

        return IKOutput(success, message, self.get_joint_names(), solution)

    def service_exotica_info(self, req):
        return self.exotica_info
