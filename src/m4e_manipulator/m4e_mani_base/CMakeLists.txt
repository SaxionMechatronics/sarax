cmake_minimum_required(VERSION 2.8.3)
project(m4e_mani_base)

find_package(catkin REQUIRED COMPONENTS
  controller_manager diagnostic_updater geometry_msgs hardware_interface
  realtime_tools roscpp sensor_msgs std_msgs)
find_package(Boost REQUIRED COMPONENTS thread chrono)

catkin_package()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
