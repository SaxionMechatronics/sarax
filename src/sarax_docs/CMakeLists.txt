cmake_minimum_required(VERSION 3.8)
project(sarax_docs)

find_package(ament_cmake REQUIRED)

install(DIRECTORY source
  DESTINATION share/${PROJECT_NAME})

install(FILES Doxyfile conf.py
  DESTINATION share/${PROJECT_NAME})

ament_package()
