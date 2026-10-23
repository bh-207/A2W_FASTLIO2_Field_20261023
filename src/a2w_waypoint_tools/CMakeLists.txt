cmake_minimum_required(VERSION 3.8)
project(a2w_waypoint_tools)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS 2 Humble on Ubuntu 22.04 is built for the system Python 3.10.  A
# concurrently active Conda environment must not redirect this package to an
# ABI-incompatible interpreter.
set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS system Python" FORCE)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(
  PYTHON_INSTALL_DIR
  "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
  CACHE INTERNAL "Python package install directory" FORCE
)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/waypoint_recorder scripts/route_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_route_storage test/test_route_storage.py)
  ament_add_pytest_test(test_capture_readiness test/test_capture_readiness.py)
  ament_add_pytest_test(test_ros_conversions test/test_ros_conversions.py)
  ament_add_pytest_test(test_terminal_input test/test_terminal_input.py)
  ament_add_pytest_test(test_route_path test/test_route_path.py)
  ament_add_pytest_test(test_route_follower_ros test/test_route_follower_ros.py TIMEOUT 90)
endif()

ament_package()
