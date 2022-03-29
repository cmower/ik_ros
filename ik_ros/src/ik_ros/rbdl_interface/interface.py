import rbdl
import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from rpbi.config import replace_package, load_config
from ik_ros.msg import RBDLProblem
from ik_ros.srv import RBDL, RBDLResponse
from ..ik_output import IKOutput
from ..ik_interface import IK

"""

Original code by Theodoros Stouraitis (@stoutheo). Adapted by Chris E. Mower for the ik_ros package (@cmower).

"""

__all__ = ['RBDLInterface']

EEBodyPointPosition = np.array([0.0, 0.0, -0.0]) #np.zeros(3)

class PyRBDLRobot:

    def __init__(self, file_name, end_effector_name, base_position, base_orientation_quat, q0):

        # Load Robot rbdl model
        # TODO: consider if we can support non floating base robots
        file_name = replace_package(file_name)
        print("-------- here")
        self.rbdlModel = rbdl.loadModel(file_name, verbose = True, floating_base = True)
        print("-------- here")

        # Get end-effector body for rbdl
        self.rbdlEndEffectorID = rbdl.Model.GetBodyId(self.rbdlModel, end_effector_name)

        # IK Variables initialization
        self.numJoints = self.rbdlModel.qdot_size
        self.rbdlEEBodyPointPosition = EEBodyPointPosition
        self.qBasePos = np.array(base_position)
        self.qBaseOrientQuat = np.array(base_orientation_quat)
        # old implementation with Euler angles loaded from file
        # ori = R.from_euler('xyz', base_orient_eulerXYZ, degrees=True)
        # self.qBaseOrientQuat = rbdl.Quaternion.toNumpy(rbdl.Quaternion.fromPythonMatrix(ori.as_matrix()))
        self.qInitial = np.array(q0)

        # ---- place robot to the base position and orientation

        # Create numpy arrays for the state
        self.q = np.zeros(self.rbdlModel.q_size)
        # Modify the state to place robot in appropriate position and orientation
        self.q[0:3] = self.qBasePos
        # following the convection of RBDL, the vector of the quaternion is in
        # slide 3:6 of the configuration (for floating base)
        # and the scalar part of the quaternion is last after all the joints
        self.q[3:6] = self.qBaseOrientQuat[0:3]
        self.q[-1] = self.qBaseOrientQuat[3]
        self.q[6:self.numJoints] = self.qInitial

        # build the name list of joints
        self.joint_name = []
        for i in range(len(self.rbdlModel.mJoints)):
            # JointTypeRevolute
            index = self.rbdlModel.mJoints[i].q_index
            # the index 3 indicates the floating base
            if index > 3:
                self.joint_name.append("Joint_"+str(index)+"_"+self.rbdlModel.mJoints[i].mJointType)

    def updateJointConfig(self, qNew):
        self.q[6:self.numJoints] = qNew

    def updateBasePos(self, posNew):
        self.q[0:3] = posNew

    def updateBaseOrientationEuler(self, orient_eulerXYZNew):
        ori_mat = np.asarray(XYZeuler2RotationMat(orient_eulerXYZNew))
        qBaseOrientQuatNew = rbdl.Quaternion.toNumpy(rbdl.Quaternion.fromPythonMatrix(ori_mat))
        self.q[3:6] = qBaseOrientQuatNew[0:3]
        self.q[-1] = qBaseOrientQuatNew[3]

    def updateBaseOrientationQuat(self, orient_quat):
        self.q[3:6] = orient_quat[0:3]
        self.q[-1] = orient_quat[3]

    def getJointConfig(self):
        return self.q[6:self.numJoints]

    def getJointName(self):
        return self.joint_name


    ### --- why use two different function for end-effector position and orientation
    def getCurEEPos(self):
        return rbdl.CalcBodyToBaseCoordinates(self.rbdlModel, self.q, self.rbdlEndEffectorID, EEBodyPointPosition)

    def getCurEEOri(self):
        return rbdl.CalcBodyWorldOrientation(self.rbdlModel, self.q, self.rbdlEndEffectorID)

    # ---------------------------------------------------------------------#
    # Test funtion
    # ---------------------------------------------------------------------#
    def testFK4baseNewPoseOrient4LWR(self):
        ''' Test forward kinematics, when the the position, orientation and
        configuration of the robot is changed. '''
        # Get first link body of the LWR arm rbdl
        body_base_ID = rbdl.Model.GetBodyId(self.rbdlModel, "lwr_arm_0_link")
        # Transform coordinates from local to global coordinates
        # define a local point w.r.t to a body
        point_local = np.array([0, 0., 0.])
        print(" Local position of the point ", point_local)
        global_point_base = rbdl.CalcBodyToBaseCoordinates (self.rbdlModel, self.q, body_base, point_local)
        print(" Global position of the point ", global_point_base)

class PyRBDL4dIK:

    def __init__(self, time_step, file_name, end_effector_name, base_position, base_orientation_quat, q0, ik_info):

        self.robot = PyRBDLRobot(file_name, end_effector_name, base_position, base_orientation_quat, q0)

        self.dt = time_step  # NOTE: this is not used

        # task indexes to switch from full 6D to only position or orienation
        self.f_indx = ik_info['f_indx']
        self.l_indx = ik_info['l_indx']

        # weights for tracking between orientation axes
        self.ori_axis_weights = ik_info['ori_axis_weights']
        # Find the unique elements of an array
        unique, index, counts = np.unique(np.array(self.ori_axis_weights), return_index=True, return_counts=True)
        # get the index of the elemnt that is 1, if it exist
        self.axis_idx = -1
        if (counts[unique==1.0]==1.0):
            self.axis_idx = index[unique==1.0][0]

        # set the function that computes the orienation error
        # error based on frames matching
        self.computeOrientationDelta = self.computeOrientationDelta3D
        # error based on vector matching
        # (match Z axis of robot with axis with index 'axis_idx')
        if self.axis_idx != -1:
            self.computeOrientationDelta = self.computeOrientationDelta1axis

        # scaling parameter between orientation and position
        self.scale_pos_orient = ik_info['scale_pos_orient']

        # parameters of the nullspace motion
        self.alpha_null = np.array(ik_info['alpha_null'])
        self.h_delta = np.deg2rad(np.array(ik_info['h_delta']))
        self.h_norm = np.deg2rad(np.array(ik_info['h_norm']))


    def fullDiffIKStep(self, globalTargetPos3D, globalTargetOri3D):
        delta = self.computeDelta(globalTargetPos3D, globalTargetOri3D)
        JG = self.computeGlobalJacobian()
        self.fullDiffIK(JG, delta)

    """ --------------- Functions made for self use -------------------------"""
    def computeOrientationDelta3D(self, orientA, orientB):
        # least square error between two frames
        error = R.align_vectors(np.transpose(orientA.as_matrix()), orientB.as_matrix(), self.ori_axis_weights)[0]
        return error.as_rotvec()

    def computeOrientationDelta1axis(self, orientA, orientB):
        # compute error only between a pair fo vectors
        zAxisTarget = orientA.as_matrix()[:,self.axis_idx] # get target axis vector
        zAxisCurrent = orientB.as_matrix()[2, :]           # get current Z axis vector
        axis = np.cross(zAxisCurrent, zAxisTarget)
        vectdot_align = zAxisTarget.dot(zAxisCurrent)
        # we clip the value of the dot product to ensure that it's between -1 to 1
        # as the vectors fo the dot product are normalized
        angle = np.arccos(np.clip(vectdot_align, -1.0, 1.0))

        if np.isnan(angle).any():
            rospy.logerr(" The is something wrong with orientantion of the IK.")

        return  axis*angle

    def computeDelta(self, globalTargetPos3D, globalTargetOri3D):
        # retrieve global position of the end-effector
        global_eePos = self.robot.getCurEEPos()
        # position error
        dpos = globalTargetPos3D - global_eePos

        # compute global orientation of the end-effector
        global_eeOri_mat = self.robot.getCurEEOri()
        global_eeOri = R.from_matrix(global_eeOri_mat)

        # globalTargetOri3D is a matrix form
        global_eeOriTarget = R.from_matrix(globalTargetOri3D)

        # compute orientation error
        dori = self.computeOrientationDelta(global_eeOriTarget, global_eeOri)

        # get delta orientation as a vector + also scale it
        doriVec = dori * self.scale_pos_orient

        # position and orientation (in euler angles) error
        # we stack first angular error and then position error because of the RBDL jacobian form (see below)
        # Computes the 6-D Jacobian $G(q)$ that when multiplied with $\dot{q}$ gives a 6-D vector
        # that has the angular velocity as the first three entries and the linear velocity as the last three entries.
        delta = np.concatenate((doriVec, dpos), axis=None)

        return delta

    def computeGlobalJacobian(self):
         # Compute Jacobian
        JG = np.zeros([6, self.robot.numJoints])
        rbdl.CalcPointJacobian6D(self.robot.rbdlModel, self.robot.q, self.robot.rbdlEndEffectorID, self.robot.rbdlEEBodyPointPosition, JG, update_kinematics=True)
        return JG

    def fullDiffIK(self, J, delta):

         # Compute Jacobian pseudo-inverse
        Jpinv = np.linalg.pinv(J)

        # select joints and dimensions
        J_arm = J[self.f_indx:self.l_indx,6:self.robot.numJoints]
        Jpinv_arm = Jpinv[6:self.robot.numJoints,self.f_indx:self.l_indx]
        # Compute differential kinematics
        dq = Jpinv_arm.dot(delta[self.f_indx:self.l_indx])

        _, c = J_arm.shape
        # retrieve current configuration
        qprev = self.robot.getJointConfig()

        # compute null-space component
        dq_nullspace_motion = (np.eye(c)-Jpinv_arm @ J_arm) @ ((self.h_delta-qprev)*(1/self.h_norm**2))

        if (abs(dq) > np.deg2rad(5)).any():
            print("---------------------------------------------------------------")
            print(" BIG STEP IN IK ")
            print("---------------------------------------------------------------")

        # compute new configuration
        qnext = qprev + dq + self.alpha_null*dq_nullspace_motion

        #  update configuration
        self.robot.updateJointConfig(qnext)


class RBDLInterface(IK):

    name = 'rbdl'
    problem_msg_type = RBDLProblem
    srv_type = RBDL
    srv_resp_type = RBDLResponse

    def __init__(self):

        # listen to tf topic for end effector targets
        self.IK_listen_buff = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.IK_listen_buff)
        self.robot_name = self.setupPyRBDLRobot(rospy.get_param('~config_filename'))

        # initialization of the Forward Kinematics
        self.target_EE_position = self.robotIK.robot.getCurEEPos()
        curOri = R.from_matrix(self.robotIK.robot.getCurEEOri())
        self.target_EE_orientation = np.transpose(curOri.as_matrix())

    def setupPyRBDLRobot(self, config_file_name):

        # Load robot configuration
        config = load_config(config_file_name)

        # Extract data from configuration
        file_name = config['file_name']
        robot_name = config['name']
        end_effector_name = config['end_effector']
        use_fixed_base = True #config['use_fixed_base']  # currently the RBDL IK interface only supports fixed base manipulators
        ik_info = config['IK']
        base_link_name = config['base_link_name']

        # Establish connection with Robot in PyBullet/Real world via ROS
        msgRobotState = rospy.wait_for_message(f"rpbi/{robot_name}/joint_states", JointState)
        # set robot to the curent configuration obtained from ros topic
        init_joint_position = list(msgRobotState.position)

        rospy.loginfo(f"{self.name}: Reading for /tf topic the position and orientation of the robot")
        # Loop till the position and orientation of the base the robots has been read.
        while 1:
            try:
                # Read the position and orientation of the robot from the /tf topic
                trans = self.IK_listen_buff.lookup_transform('rpbi/world', base_link_name, rospy.Time())
                break
            except:
                rospy.logwarn(f"{self.name}: /tf topic does NOT have {base_link_name}")
        # replaces base_position = config['base_position']
        base_position = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        # replaces: base_orient_eulerXYZ = config['base_orient_eulerXYZ']
        base_orient_quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        # base_orient_eulerXYZ = config['base_orient_eulerXYZ']

        # Create pybullet robot instance
        dt = None  # NOTE: this variable is not used in the PyRBDL4dIK class
        self.robotIK = PyRBDL4dIK(dt, file_name, end_effector_name, base_position, base_orient_quat, init_joint_position, ik_info)

        return robot_name

    def get_joint_names(self):
        return self.robotIK.robot.getJointName()

    def solve(self, problem):

        success = True
        message = 'solved RBDL IK problem'

        # Extract transform from problem
        transform = problem.target_EE_transform
        target_EE_position = np.asarray([transform.translation.x, transform.translation.y,transform.translation.z])
        target_EE_ori = R.from_quat(np.asarray([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]))
        target_EE_orientation = target_EE_ori.as_matrix()

        # Update current joint position
        self.robotIK.robot.updateJointConfig(np.array(self.resolve_joint_position_order(problem.current_position)))

        # Solve problem and update robotik
        try:
            self.robotIK.fullDiffIKStep(self.target_EE_position, self.target_EE_orientation)
            solution = self.robotIK.robot.getJointConfig().tolist()
        except Exception as err:
            success = False
            message = 'failed to solve RBDL IK problem'
            solution = []

        return IKOutput(success, message, self.get_joint_names(), solution)
