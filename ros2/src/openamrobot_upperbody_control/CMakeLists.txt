# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.8)
project(openamrobot_upperbody_control)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/openamrobot_upperbody_control)
# Issue #44: the simulated lift guard (mock and gazebo only).
install(PROGRAMS scripts/lift_guard.py scripts/lift_guard_odom_stub.py scripts/park_arms.py scripts/wait_for_lift_guard.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(arm_controllers_yaml test/test_arm_controllers_yaml.py TIMEOUT 300)
  # Issue #44: the lift guard's decision, every condition and every missing, stale or malformed input; no ROS runtime.
  ament_add_pytest_test(lift_guard_logic test/test_lift_guard_logic.py TIMEOUT 120)
  # Issue #44, review of #53: the node with rclpy. Out-of-order controller-manager answers (in process, deterministic) and the exit on
  # SIGTERM/SIGINT (a child process against a stand-in controller manager), in its own ROS domain.
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_pytest_test(lift_guard_node test/test_lift_guard_node.py TIMEOUT 180)
endif()

ament_package()
