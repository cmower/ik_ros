# Pybullet interface

The Pybullet interface relies on functionality provided in the [ROS-Pybullet Interface](https://github.com/cmower/ros_pybullet_interface).
Thus, the interface to Pybullet IK methods does not require the solver node.
In your launch file, you only need to add the setup node.

**NOTE**: in the robot configuration file for the Pybullet interface, you must set the [`start_ik_callback`](https://github.com/cmower/ros_pybullet_interface/blob/0b6ca7b87ca58b656eb728f1f076f5133fdbcd5b/ros_pybullet_interface/src/rpbi/pybullet_robot_ik.py#L14-L16) to `True`.
