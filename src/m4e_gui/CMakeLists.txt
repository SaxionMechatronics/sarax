cmake_minimum_required(VERSION 2.8.3)
project(m4e_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/m4e_gui_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
