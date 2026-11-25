cmake_minimum_required(VERSION 3.8)
project(sarax_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes rviz launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_camera_extrinsic test/test_camera_extrinsic.py)
  ament_add_pytest_test(test_joint_limits test/test_joint_limits.py)
endif()

ament_package()
