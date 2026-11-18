cmake_minimum_required(VERSION 3.10)
project(sarax_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters surface search features)
find_package(pcl_conversions REQUIRED)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(cloud_filter_node src/cloud_filter_node.cpp)
target_include_directories(cloud_filter_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(cloud_filter_node ${PCL_LIBRARIES})
ament_target_dependencies(cloud_filter_node rclcpp sensor_msgs pcl_conversions)

install(TARGETS cloud_filter_node DESTINATION lib/${PROJECT_NAME})

foreach(node mls_normal_node pca_normal_node)
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(${node} ${PCL_LIBRARIES})
  ament_target_dependencies(${node}
    rclcpp sensor_msgs visualization_msgs geometry_msgs pcl_conversions)
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/normal_selector_v1.py DESTINATION lib/${PROJECT_NAME})

ament_package()
