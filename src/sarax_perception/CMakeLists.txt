cmake_minimum_required(VERSION 3.10)
project(sarax_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(cloud_filter_node src/cloud_filter_node.cpp)
target_include_directories(cloud_filter_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(cloud_filter_node ${PCL_LIBRARIES})
ament_target_dependencies(cloud_filter_node rclcpp sensor_msgs pcl_conversions)

install(TARGETS cloud_filter_node DESTINATION lib/${PROJECT_NAME})

ament_package()
