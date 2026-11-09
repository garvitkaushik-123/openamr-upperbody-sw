# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.8)
project(openamrobot_upperbody_control)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/openamrobot_upperbody_control)
# Issue #44: the simulated lift guard (mock and gazebo only).
install(PROGRAMS scripts/lift_guard.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(arm_controllers_yaml test/test_arm_controllers_yaml.py TIMEOUT 300)
  # Issue #44: the lift guard's decision, every condition and every missing, stale or malformed input; no ROS runtime.
  ament_add_pytest_test(lift_guard_logic test/test_lift_guard_logic.py TIMEOUT 120)
endif()

ament_package()
