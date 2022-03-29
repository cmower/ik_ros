# ik_ros

The main implementation is contained in this package.

# Nodes

## `ik_setup_node.py INTERFACE_NAME`

The setup node packs the IK problem and continuously publishes the message at a given sampling frequency.

### Publishers

* `ik` (topic type determined by interface, see `ik_ros/msg`)

  IK problem message.

### Subscribers

Subscribers are interface specific.

#### Trac IK setup node subscribers

* `ik/setup/trac_ik/qinit` ([sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html))

  Initial status of the joints as seed.

* `ik/setup/trac_ik/bx` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))

  X allowed bound (initial value 1e-5).

* `ik/setup/trac_ik/by` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))

  Y allowed bound (initial value 1e-5).

* `ik/setup/trac_ik/bz` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))

  Z allowed bound (initial value 1e-5).

* `ik/setup/trac_ik/brx` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))

  rotation over X allowed bound (initial value 1e-3).

* `ik/setup/trac_ik/bry` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))

  rotation over Y allowed bound (initial value 1e-3).

* `ik/setup/trac_ik/brz` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))

  rotation over Z allowed bound (initial value 1e-3).

#### EXOTica setup node subscribers

* `ik/setup/exotica/task_map_goal/TASK_MAP_NAME`  ([std_msgs/Float64MultiArray](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64MultiArray.html))

  A subscriber will be initialized for each task map goal. The goal must be represented as a `std_msgs/Float64MultiArray` message and be the correct length - you can get task space dimension (i.e. `nrho`) using the `exotica_info` service (see below). A useful node to make use of here is `tf_to_floatarray_node.py` (see below).

* `ik/setup/exotica/previous_solution` ([sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html))

  The previous joint state solutions are collected on this topic. These are used in EXOTica joint motion smoothing task maps (e.g. JointVelBackwardDifference).

* `ik/setup/exotica/start_state` ([sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html))

  Start state for the optimizer.

#### Pybullet setup node subscriber

* `ik/setup/pybullet/currentPosition` ([sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html))

    list of joint positions. By default PyBullet uses the joint positions of the body. If provided, the targetPosition and targetOrientation is in local space!

### Parameters

#### General for all interfaces

All interfaces take the following parameters.

* `~start_on_init` (bool, default: false)

  When true, the node will start on initialization.

* `~hz` (int, default 50)

  Sampling frequency for IK problem messages.

#### Interface specific

Parameters specific to each interface are described in the following.

##### Trac IK setup node parameters

* `~child_frame_id` (str)

  Frame ID for the child frame. This should be the target frame for the IK.

* `~parent_frame_id` (str)

  Frame ID for the parent frame.

##### EXOTica setup node parameters

* `~max_num_prev_solutions` (int, default: 1)

  Number of previous solutions to keep in log.

* `~sync_tf_to_exotica_object` (list[str], default: empty-list)

  A list of EXOTica links to be synced with TF frames. Each element of the list should have format: `"tf_parent_frame, tf_child_frame, exo_parent_frame, exo_child_frame"`

##### Pybullet setup node parameters

The following has mainly been taken from the [Pybullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#).

* `~config/robot_name` (str)

  Name of robot in ROS-Pybullet interface.

* `~config/link_name` (str)

  Name of end-effector link.

* `~config/lowerLimits` (list[float])

  Optional null-space IK requires all 4 lists (lowerLimits, upperLimits, jointRanges, restPoses). Otherwise regular IK will be used. Only provide limits for joints that have them (skip fixed joints), so the length is the number of degrees of freedom. Note that lowerLimits, upperLimits, jointRanges can easily cause conflicts and instability in the IK solution. Try first using a wide range and limits, with just the rest pose.

* `~config/upperLimits` (list[float])

  Optional null-space IK requires all 4 lists (lowerLimits, upperLimits, jointRanges, restPoses). Otherwise regular IK will be used.. lowerLimit and upperLimit specify joint limits.

* `~config/jointRanges` (list[float])

  Optional null-space IK requires all 4 lists (lowerLimits, upperLimits, jointRanges, restPoses). Otherwise regular IK will be used.

* `~config/resetPoses` (list[float])

  Optional null-space IK requires all 4 lists (lowerLimits, upperLimits, jointRanges, restPoses). Otherwise regular IK will be used.. Favor an IK solution closer to a given rest pose.

* `~config/jointDamping` (list[float])

  jointDamping allow to tune the IK solution using joint damping factors.

* `~config/solver` (int)

 0 (p.IK_DLS) or 1 (p.IK_SDLS), Damped Least Squares or Selective Damped Least Squares, as described in the paper by Samuel Buss "Selectively Damped Least Squares for Inverse Kinematics".

* `~config/maxNumIterations` (int)

  Refine the IK solution until the distance between target and actual end effector position is below this threshold, or the maxNumIterations is reached. Default is 20 iterations.

* `~config/residualThreshold` (double)

  Refine the IK solution until the distance between target and actual end effector position is below this threshold, or the maxNumIterations is reached.

* `~config/child_frame_id` (str)

  Frame ID for the child frame. This should be the target frame for the IK.

* `~config/parent_frame_id` (str)

  Frame ID for the parent frame.

### Service

* `ik/setup/INTERFACE_NAME` ([SetBool](http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html))

  Toggle the node on/off (true/false respectively).

## `ik_solver_node.py INTERFACE_NAME`

The setup node listens for IK problems, solves the problem using the specified IK interface, and publishes the solution.
**Note**, there is no solver interface for Pybullet since the solving is performed in the ROS-Pybullet interface.

### Subscriber

* `ik` (topic type determined by interface, see `ik_ros/msg`)

  IK problem message.

### Publisher

* `joint_states/target` ([sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html))

  The target joint state solution.

* `ik/solution` ([ik_ros/IKSolution](https://github.com/cmower/ik_ros/blob/master/ik_ros/msg/IKSolution.msg))

  The IK solution with additional meta information.

### Parameters

#### General for all interfaces

* `~start_on_init` (bool, default: false)

  When true, the node will start on initialization.

#### Interface specific

##### Trac IK solver node parameters

* `~urdf_filename` (str)

  The filename for the URDF.

* `~base_link` (str)

  Starting link of the chain.

* `~tip_link` (str)

  Last link of the chain.

* `~timeout` (double, min: 0.0, max: inf, default: 0.005)

  Timeout in seconds for the IK calls.

* `~epsilon` (double, min: 0.0, max: inf, default: 1e-5)

  Error epsilon.

* `~solver_type` (str, default: `'Speed'`)

  Type of solver, can be: Speed (default), Distance, Manipulation1, Manipulation2.

##### EXOTica solver node parameters

* `~xml_filename` (str)

  Filename for the [XML initialization file](https://ipab-slmc.github.io/exotica/XML.html).

* `~use_scipy_solver` (bool, default: false)

  When true the [EXOTica scipy solver](https://github.com/ipab-slmc/exotica/tree/master/exotations/solvers/exotica_scipy_solver) is used.

* `~scipy_solver_method` (str, default: `'SLSQP'`)

  Type of solver, see also [scipy documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html).

### Service

#### General for all interfaces

* `ik/solver/INTERFACE_NAME/toggle` ([SetBool](http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html))

  Toggle the node on/off (true/false respectively).

* `ik/solver/INTERFACE_NAME/solve` (service type determined by interface, see `ik_ros/srv`)

  Given the problem in the service request, solve the problem and return the solution in the service response.

* `ik/solver/INTERFACE_NAME/joint_names` ([ik_ros/JointNames](https://github.com/cmower/ik_ros/blob/master/ik_ros/srv/JointNames.srv))

  Return the list of joint names in the order defined by the given interface.

#### Interface specific

##### EXOTica solver node service

* `exotica_info` ([EXOTicaInfo](https://github.com/cmower/ik_ros/blob/master/ik_ros/srv/EXOTicaInfo.srv))

  Summary information for EXOTica configuration.

## `tf_to_floatarray_node.py`

This node maps a given tf frame to a `std_msgs/Float64MultiArray` message.

### Publisher

* `transform` ([std_msgs/Float64MultiArray](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64MultiArray.html))

  Data array containing transform.

### Parameters

* `~child_frame_id` (str)

  Frame ID for the child frame.

* `~parent_frame_id` (str)

  Frame ID for the parent frame.

* `~hz` (int, default: 50)

  Sampling frequency.

* `~mode` (str, default `pos+quat`)

  Determines format of output data.
  * `~mode` is `pos+quat`: data array with 7 elements representing position (3) and quaternion (4)
  * `~mode` is `pos+eul`: data array with 6 elements representing position (3) and Euler angles (3)
  * `~mode` is `pos`: data array with 3 elements representing position (3)
