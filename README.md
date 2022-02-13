# ik_ros

Inverse kinematics integration for ROS. This package provides a
standardized interface for IK solvers and ROS. The input to the node
must have type `std_msgs/Float64MultiArray` that defines the goal and
parameters for the IK solver, the output of the node is a
`sensor_msgs/JointState` message containing the solution. When
initialized, the node sits waiting to be started by a call to a
service. The services either start a streaming joint state of
solutions at a given frequency, a callback that streams solutions at
the same frequency it recieves them (assuming the IK solves within the
sampling time), or responds directly to the service call. Several
interfaces exist already:
* [trac_ik](https://bitbucket.org/traclabs/trac_ik.git)
* [EXOTica](https://github.com/ipab-slmc/exotica)

# Adding a new IK interface

You must implement a class that inherits from the `IK` class found in
`ik_ros/src/ik_ros/ik.py`. This must define several methods:
* `reset`: reset IK problem/solver, must be called prior to solve. Note the setup parameter must be of type `std_msgs/Float64MultiArray`
* `solve`: calls the IK solver.
* `joint_names`: return a list of joint names in same order as solution.
* `solution`: returns the solution for the previous call to solve as a Python list.

Add the new script to `ik_ros/src/ik_ros`, and make sure to import the
class in `ik_ros/scripts/ik_node.py`.

# IK node

For the main `ik_node.py`, the parameters depend on the given command
line arguments that specify the IK interface. Note, the first section
below applies to all IK interfaces, following describes the parameters
required for each individual IK interface and the format for the setup
message. Other nodes included in this package are described below.

EXOTica specifies the IK using an XML configuration file. This means
that the `reset` method must be tailored to each configuration. A base
EXOTica class is implemented in `ik_ros/src/ik_ros/exotica.py`.

## `ik_node.py`

The following sections apply to all IK interfaces.

### Subscribed topics

* `setup` ([std_msgs/Float64MultiArray](http://docs.ros.org/en/api/std_msgs/html/msg/Float64MultiArray.html))

    Goal and parameters for the IK problem. The format for this is defined in the `reset` method used in the IK interface.

### Published topics

* `joint_states/target` ([sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html))

    The target joint state.

## `ik_node.py TracIK`

### Parameters

* `~base_link` (string)

    Starting link of the chain.

* `~tip_link` (string)

    Last link of the chain.

* `~urdf_filename` (string)

    Filename for the URDF. Note, you can use a relative path to ROS
    packages. E.g. `{ros_package}/path/to/file.urdf`.

* `~timeout` (double, default: 0.005, min: 0.0)

    Timeout in seconds for the IK calls.

* `~epsilon` (double, default: 1e-5, min: 0.0)

    Error epsilon.

* `~solve_type` (string, default: Speed)

    Type of solver, can be: Speed, Distance, Manipulation1, Manipulation2.

* `~bx` (float, default 1e-5)

    X allowed bound.

* `~by` (float, default 1e-5)

    Y allowed bound.

* `~bz` (float, default 1e-5)

    Z allowed bound.

* `~brx` (float, default 1e-3)

    Rotation over X allowed bound.

* `~bry` (float, default 1e-3)

    Rotation over Y allowed bound.

* `~brz` (float, default 1e-3)

    Rotation over Z allowed bound.

### Setup format

Note, in the following `NDOF` stands for the number of degrees of freedom.

* `setup.data[0]`: X-goal
* `setup.data[1]`: Y-goal
* `setup.data[2]`: Z-goal
* `setup.data[3]`: RX-goal
* `setup.data[4]`: RY-goal
* `setup.data[5]`: RZ-goal
* `setup.data[6]`: RW-goal
* `setup.data[7:]`: initial joint configuration, this should have dimension equal to `NDOF`.

## `ik_node.py SingleArmExoticaEffFrame`

### Parameters


* `~exotica_xml_filename` (string)

    EXOTica [XML Initialization](https://ipab-slmc.github.io/exotica/XML.html) configuration file. Note, you can use a relative path to ROS
    packages. E.g. `{ros_package}/path/to/file.xml`.

* `~use_scipy_solver` (bool, default: false)

    When true EXOTica [Scipy solver](https://github.com/ipab-slmc/exotica/tree/master/exotations/solvers/exotica_scipy_solver) is used, False means the solver defined in the XML configuration is used.

* `~scipy_solver_method` (string, default; SLSQP)

    Defines the solver used by the `scipy.optimize.minimize` method, see [documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html).

* `~target_frame_id` (string)

    Target frame defined in the EXOTica XML configuration file.

* `~base_frame_id` (string)

    Base frame defined in the EXOTica XML configuration file.

### Setup format

* `setup.data`, when `len(setup.data) = 3`: position
* `setup.data`, when `len(setup.data) = 6`: position + rotation (as Euler angles)
* `setup.data`, when `len(setup.data) = 7`: position + rotation (as quaternion)

# Other Nodes

## `tf_to_floatarray_node.py`

This node does not subscribe to any topic. It listens for a given
transform broadcasted using the [`tf2`](http://wiki.ros.org/tf2)
library.

### Published topics

* `transform` ([std_msgs/Float64MultiArray](http://docs.ros.org/en/api/std_msgs/html/msg/Float64MultiArray.html))

    A transform with format `[x,y,z,rx,ry,rz,rw]`.

### Parameters

* `~child_frame_id` (string)

    The child frame for the transform.


* `~parent_frame_id` (string)

    The parent frame for the transform.

* `~hz` (integer, default: 50)

    The sampling frequency in Herts.
