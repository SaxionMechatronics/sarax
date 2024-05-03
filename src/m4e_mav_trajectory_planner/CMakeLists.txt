cmake_minimum_required(VERSION 3.0.2)
project(m4e_mav_trajectory_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_trajectory_generation
  mav_trajectory_generation_ros
  nav_msgs
  message_generation
)

add_service_files(
  FILES
  PlanTakeoff.srv 
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES m4e_mav_trajectory_planner
  CATKIN_DEPENDS std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

############
# BINARIES #
############
add_executable(m4e_mav_trajectory_planner
        src/example_planner.cpp
        src/example_planner_node.cpp
)

add_dependencies(m4e_mav_trajectory_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(m4e_mav_trajectory_planner
${catkin_LIBRARIES}
)
