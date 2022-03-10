cmake_minimum_required(VERSION 3.0.2)
project(ik_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2
  tf2_ros
  sensor_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  EXOTicaProblem.msg
  TracIKProblem.msg
)

add_service_files(
  FILES
  EXOTica.srv
  TracIK.srv
  JointNames.srv
  EXOTicaInfo.srv
  )

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS rospy tf2 tf2_ros message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/ik_node.py
  scripts/exotica_set_problem_node.py
  scripts/trac_ik_set_problem_node.py
  scripts/tf_to_floatarray_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
