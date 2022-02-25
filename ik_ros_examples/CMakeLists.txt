cmake_minimum_required(VERSION 3.0.2)
project(ik_ros_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2
  tf2_ros
  geometry_msgs
  )

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/figure_eight_node.py
  scripts/trac_ik_example_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
