cmake_minimum_required(VERSION 3.10)
project(sarax_px4_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

ament_python_install_package(${PROJECT_NAME})

add_executable(state_bridge src/state_bridge.cpp)
target_include_directories(state_bridge PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(state_bridge rclcpp px4_msgs nav_msgs geometry_msgs tf2_ros)

install(TARGETS state_bridge DESTINATION lib/${PROJECT_NAME})
# frames.hpp is a public header: the vision-repo's PX4 modes include it.
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_frames test/test_frames.cpp)
  target_include_directories(test_frames PRIVATE ${EIGEN3_INCLUDE_DIR})
  # robostack's launch_testing pytest plugins are incompatible with its pytest; the frame
  # tests need no plugin, so skip entrypoint autoload.
  ament_add_pytest_test(test_frames_py test/test_frames.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()

ament_package()
