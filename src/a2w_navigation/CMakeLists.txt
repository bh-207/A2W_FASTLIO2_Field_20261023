cmake_minimum_required(VERSION 3.8)
project(a2w_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/localization_guard
    scripts/apriltag_nav_goal
    scripts/nav2_activation_gate
    scripts/navigation_runtime_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY behavior_trees config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_package_contract test/test_package_contract.py)
  ament_add_pytest_test(test_nav2_configuration test/test_nav2_configuration.py)
  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)
  ament_add_pytest_test(test_localization_guard_policy test/test_localization_guard_policy.py)
  ament_add_pytest_test(test_runtime_monitoring test/test_runtime_monitoring.py)
  ament_add_pytest_test(test_motion_authorization_ros test/test_motion_authorization_ros.py)
  ament_add_pytest_test(test_apriltag_goal_policy test/test_apriltag_goal_policy.py)
  ament_add_pytest_test(test_apriltag_nav_goal_node test/test_apriltag_nav_goal_node.py)
endif()

ament_package()
