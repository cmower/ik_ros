cmake_minimum_required(VERSION 3.0.2)
project(ik_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2
  tf2_ros
  sensor_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  ToggleIK.srv
  SolveIK.srv
  JointNameOrder.srv
  )

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs
)

catkin_package(
 CATKIN_DEPENDS rospy tf2 tf2_ros message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/ik_node.py
  scripts/tf_to_floatarray_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
