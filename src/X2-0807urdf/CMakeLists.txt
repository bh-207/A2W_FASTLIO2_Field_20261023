cmake_minimum_required(VERSION 3.8)
project(a2w_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES urdf/a2w_navigation.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_a2w_description test/test_a2w_description.py)
  ament_add_pytest_test(test_fixed_tf_ros test/test_fixed_tf_ros.py)
endif()

ament_package()
