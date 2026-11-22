# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.8)
project(openamrobot_upperbody_gazebo)
find_package(ament_cmake REQUIRED)
install(DIRECTORY worlds models launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # how sim.launch.py stops the simulator, on decoy processes and on the launch description (P17); no Gazebo, no ROS graph
  ament_add_pytest_test(sim_launch_shutdown test/test_sim_launch_shutdown.py TIMEOUT 120)
  # how sim.launch.py starts the controllers: the broadcaster's switch timeout and the chain on its exit code (16 Oct); no Gazebo
  ament_add_pytest_test(sim_launch_spawners test/test_sim_launch_spawners.py TIMEOUT 120)
  ament_add_pytest_test(sim_spawn_height test/test_sim_spawn_height.py TIMEOUT 120)
  # the simulated arm controllers update at the upstream rate: the launch passes the overlay, and the world step divides its period; no Gazebo
  ament_add_pytest_test(sim_control_rate test/test_sim_control_rate.py TIMEOUT 120)
  # G01's rule for who reports lift_joint (issue #17): once by joint_state_broadcaster on mock/gazebo, only the merger's default on real
  ament_add_pytest_test(g01_lift_source test/test_g01_lift_source.py TIMEOUT 60)
  # G01 starts a real Gazebo server and a controller manager. It picks its own ROS_DOMAIN_ID and GZ_PARTITION (see the test),
  # so it cannot see or be seen by another stack or simulator on the machine. 90 s is the budget for startup and every assertion.
  add_launch_test(test/test_g01_gazebo_smoke.py TIMEOUT 90)
  # G02 guards the finger-mimic decision (README). It passes the Bullet-Featherstone engine explicitly: the launch defaults to DART, on which it fails.
  add_launch_test(test/test_g02_gripper_mimic.py TIMEOUT 90)
endif()

ament_package()
