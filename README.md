# ik_ros

Inverse kinematics integration for ROS. This package provides a
standardized interface for IK solvers and ROS. Given an interface to a
solver, the `ik_node.py` script will listen to goals on the `ik` topic
with type defined by the interface, then output goals in a callback
loop. Additionaly, you can use the node as a service if you do not
need to stream IK goals. The output of the node is a
`sensor_msgs/JointState` message on the topic `joint_states/target`
containing the solution.

Supported interfaces:
* [trac_ik](https://bitbucket.org/traclabs/trac_ik.git)
* [EXOTica](https://github.com/ipab-slmc/exotica)
* [RBDL](https://rbdl.github.io/) (in development)


# Adding a new IK interface

You must implement a class that inherits from the `IK` class found in
`ik_ros/src/ik_ros/ik_interface.py`. This must define several methods
and attributes (below). Add the new script to `ik_ros/src/ik_ros`, and make
sure to import the class in `ik_ros/scripts/ik_node.py`.
