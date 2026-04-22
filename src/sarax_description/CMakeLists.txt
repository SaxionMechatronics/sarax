cmake_minimum_required(VERSION 3.8)
project(sarax_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes rviz launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
