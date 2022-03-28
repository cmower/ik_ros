# ik_ros

Inverse kinematics integration for ROS.
This package provides a standardized interface for IK solvers and ROS.

# System overview

![alt text](https://github.com/cmower/ik_ros/blob/master/doc/ik_ros_sys_overview.png?raw=true)

The figure above shows an overview for `ik_ros`.
Two nodes are provided (green) that setup the problem, and solve the problem.
A setup node collects the required data and packs the problem parameters in a ROS message.
The message is passed to the solver node at a given sampling frequency.
Finally, the solver node takes the problem and passes it to an IK solver.
The solution is then published to ROS.
Note, both a `ik_ros/IKSolution` and `sensor_msgs/JointState` message are published after each solve.
In addition, the solver node includes a service for solving IK problems that takes a problem as input and response with the joint state target solution.
Several interfaces are already supported for common solvers (see below).
The package has been designed to be extensible so additional solvers or custom IK solver interfaces can be easily added by implementing an interface and solver node - see the next section for details.

# Adding a new interface

To add a new interface you need to implement the following:
1. Add an IK problem message to `ik_ros/msg` (remember to include in `ik_ros/CMakeLists.txt`)
    * this must contain a header, i.e. add the line `std_msgs/Header header`
    * this file should contain the relevant problem parameters
    * typically this is variables that change at solve time
    * do not include solver initialization variables here, these should be parameters set in the IK solver node
2. Add a directory to `ik_ros/src/ik_ros` with the name of the interface.
3. In the new directory implement the following files
    * `__init__.py`, an empty file for packaging
    * `setup_node.py`, implement a class that inherits from the `IKSetupNode` in `ik_ros/src/ik_ros/ik_setup_node.py`. The goal of this class is to setup the relevant subscribers and tf listeners to pack the IK problem. Note, there are abstract methods that must be implemented that come from the `IKSetupNode` class. Also note, you must call `post_init` at the end of the `__init__` method.
    * `interface.py`, implement a class that inherits from the `IK` in `ik_ros/src/ik_ros/ik_interface.py`. The goal of this class is to provide the interface for the IK solver. Note, this is not a ROS node, but will be instantiated within a ROS node so `rospy` functionality is available to use (e.g. `rospy.get_param`). Also, there are abstract methods that must be implement that come from the `IK` parent class.
4. Add your interface to the `solver_interfaces` variable in `ik_ros/src/ik_ros/ik_solver_interfaces.py`.

# Supported interfaces

* [trac_ik](https://bitbucket.org/traclabs/trac_ik.git)
* [EXOTica](https://github.com/ipab-slmc/exotica)
* [RBDL](https://rbdl.github.io/) (in development)
* [pybullet](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.9i02ojf4k3ve)
    * *Note*, this uses the [ROS-Pybullet Interface](https://github.com/cmower/ros_pybullet_interface)
