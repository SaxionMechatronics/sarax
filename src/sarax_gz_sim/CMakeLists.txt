cmake_minimum_required(VERSION 3.8)
project(sarax_gz_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models worlds
  DESTINATION share/${PROJECT_NAME})

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_resource_path.sh.in")

ament_package()
