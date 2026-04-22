cmake_minimum_required(VERSION 3.16)
project(sarax_impedance_controller CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(sarax_msgs REQUIRED)

include_directories(include)

# -------- ImpedanceLaw plugins (separate shared lib) --------
add_library(sarax_impedance_laws SHARED
  src/laws/pd_law.cpp
  src/laws/pd_gravity_law.cpp
)
target_include_directories(sarax_impedance_laws PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sarax_impedance_laws
  rclcpp
  pluginlib
  pinocchio
)
target_link_libraries(sarax_impedance_laws Eigen3::Eigen)

# -------- Controller plugin (separate shared lib) --------
add_library(sarax_impedance_controller SHARED
  src/joint_impedance_controller.cpp
)
target_include_directories(sarax_impedance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sarax_impedance_controller
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  trajectory_msgs
  std_msgs
  rcl_interfaces
  sarax_msgs
)
target_link_libraries(sarax_impedance_controller Eigen3::Eigen)

# -------- pluginlib manifests --------
pluginlib_export_plugin_description_file(
  controller_interface sarax_controllers.xml)
pluginlib_export_plugin_description_file(
  sarax_impedance_controller sarax_impedance_laws.xml)

# -------- install --------
install(TARGETS sarax_impedance_laws sarax_impedance_controller
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include)
install(FILES sarax_controllers.xml sarax_impedance_laws.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(sarax_impedance_laws sarax_impedance_controller)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle controller_interface hardware_interface pluginlib
  realtime_tools trajectory_msgs std_msgs sarax_msgs)

# -------- tests --------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pd_law test/test_pd_law.cpp)
  target_link_libraries(test_pd_law sarax_impedance_laws)
endif()

ament_package()
