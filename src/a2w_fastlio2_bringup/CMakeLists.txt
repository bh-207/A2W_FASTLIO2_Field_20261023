cmake_minimum_required(VERSION 3.8)
project(a2w_fastlio2_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_navigation_launch_contract
    test/test_navigation_launch_contract.py
  )
endif()

ament_package()
