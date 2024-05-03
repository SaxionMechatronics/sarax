cmake_minimum_required(VERSION 3.0.2)
project(tmotor_ak60_drivers_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  socketcan_bridge
  socketcan_interface
  message_generation
  std_msgs
  sensor_msgs
  hardware_interface
  controller_manager

)

add_message_files(
  FILES
  MotorCommand.msg
  MotorOutput.msg
)

add_service_files(
  FILES
  TriggerMotor.srv
  SetMotorVal.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tmotor_ak60_drivers_cpp
  CATKIN_DEPENDS roscpp rospy socketcan_bridge socketcan_interface std_msgs message_generation hardware_interface controller_manager sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(${PROJECT_NAME}_node src/tmotor_ak60_drivers_cpp_node.cpp)
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
#target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#)

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(m4e_ros_controller_node src/m4e_ros_controller_node.cpp src/tmotor_ak60_drivers_cpp_node.cpp)
add_dependencies(m4e_ros_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(m4e_ros_controller_node
   ${catkin_LIBRARIES}
)
