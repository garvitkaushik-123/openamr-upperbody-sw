# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)
project(openamrobot_openarm_emulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Threads REQUIRED)

# The emulator itself: frames, motors, bus timing, SocketCAN. No ROS in it.
add_library(openarm_emulator_core SHARED
  src/codec.cpp
  src/motor_model.cpp
  src/bus_timing.cpp
  src/arm_emulator.cpp)
target_include_directories(openarm_emulator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(openarm_emulator_core PUBLIC Threads::Threads)

# The ros2_control controller that connects one arm's emulator to the joints Gazebo simulates.
add_library(openarm_emulator_controller SHARED src/emulator_controller.cpp)
target_link_libraries(openarm_emulator_controller PUBLIC openarm_emulator_core)
ament_target_dependencies(openarm_emulator_controller PUBLIC
  controller_interface diagnostic_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(controller_interface openarm_emulator_controllers.xml)

install(TARGETS openarm_emulator_core openarm_emulator_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch urdf worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/setup_vcan.sh scripts/inject_fault.py scripts/example_arm_trajectory.py scripts/count_sil_tests.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # The upstream CAN library, built from source at the pin (config/sil_pins.yaml). Without it the comparison with the
  # upstream layout cannot run, and the test says BLOCKED instead of passing.
  find_package(OpenArmCAN QUIET)

  # Frame layout: fixed byte vectors, and bit-for-bit against openarm_can's own encoder and decoder.
  ament_add_gtest(test_codec test/test_codec.cpp TIMEOUT 60)
  target_link_libraries(test_codec openarm_emulator_core)
  if(OpenArmCAN_FOUND)
    target_link_libraries(test_codec OpenArmCAN::openarm_can)
    target_compile_definitions(test_codec PRIVATE HAVE_OPENARM_CAN=1)
  else()
    message(WARNING "OpenArmCAN not found: the upstream layout comparison in test_codec reports BLOCKED")
  endif()

  # Motor state machine, faults, supply voltage and bus timing. No CAN interface, no ROS graph.
  ament_add_gtest(test_motor_model test/test_motor_model.cpp TIMEOUT 60)
  target_link_libraries(test_motor_model openarm_emulator_core)

  # The real upstream driver (openarm_hardware/OpenArmHW, loaded as a plugin) against the emulator on vcan_arm_l and
  # vcan_arm_r. Needs the two vcan interfaces (scripts/setup_vcan.sh) and the driver built at the pin: without either,
  # every case reports BLOCKED (a gtest skip), never a pass. One at a time: they share the interfaces.
  ament_add_gtest(test_real_driver test/test_real_driver.cpp TIMEOUT 300)
  target_link_libraries(test_real_driver openarm_emulator_core)
  ament_target_dependencies(test_real_driver hardware_interface pluginlib rclcpp rclcpp_lifecycle)

  # Pins, configuration and launch description: static checks, no CAN, no Gazebo.
  ament_add_pytest_test(test_package_static test/test_package_static.py TIMEOUT 120)

  # The whole launch, headless: Gazebo, both emulators, the real driver, the example trajectory, one E-stop power cut.
  # BLOCKED without vcan or without the driver.
  add_launch_test(test/test_sil_launch.py TARGET test_sil_launch TIMEOUT 300)

  set_tests_properties(test_real_driver test_sil_launch PROPERTIES RESOURCE_LOCK openarm_vcan RUN_SERIAL TRUE)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(openarm_emulator_core openarm_emulator_controller)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface diagnostic_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
