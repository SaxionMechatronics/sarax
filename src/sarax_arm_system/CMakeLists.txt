cmake_minimum_required(VERSION 3.16)
project(sarax_arm_system LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(transmission_interface REQUIRED)

# Transmission mapping alone (no hardware); the unit test links this.
add_library(sarax_arm_transmission SHARED src/arm_transmission.cpp)
target_include_directories(sarax_arm_transmission PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sarax_arm_transmission hardware_interface transmission_interface)

add_library(${PROJECT_NAME} SHARED src/sarax_arm_system.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface lifecycle_msgs pluginlib rclcpp rclcpp_lifecycle)
target_link_libraries(${PROJECT_NAME} sarax_arm_transmission)
pluginlib_export_plugin_description_file(hardware_interface sarax_hardware.xml)

install(TARGETS sarax_arm_transmission ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES sarax_hardware.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_arm_transmission test/test_arm_transmission.cpp)
  target_link_libraries(test_arm_transmission sarax_arm_transmission)
endif()

ament_package()
