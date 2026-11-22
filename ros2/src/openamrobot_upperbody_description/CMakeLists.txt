# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.8)
project(openamrobot_upperbody_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes config launch test DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/datums_report.py
  scripts/print_config_id.py
  scripts/check_urdf_matrix.py
  scripts/build_evidence.py
  scripts/evidence_common.py
  scripts/launch_guard.py
  scripts/harness_canary.py
  scripts/verify_a6.sh
  scripts/verify_gazebo.sh
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)

  # ament_add_pytest_test only warns when pytest is missing, which would leave
  # an empty suite reported as "0 failures". Refuse that instead.
  ament_has_pytest(_has_pytest QUIET)
  if(NOT _has_pytest)
    message(FATAL_ERROR
      "pytest not importable by ${Python3_EXECUTABLE}; tests would be silently empty")
  endif()

  # Every configuration combination: xacro expansion and frame poses, no ROS graph.
  ament_add_pytest_test(test_upperbody_configurations test/test_upperbody_configurations.py
    TIMEOUT 300)
  # Every buildable combination published by robot_state_publisher and read back from TF.
  ament_add_ros_isolated_pytest_test(test_upperbody_tf test/test_upperbody_tf.py
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST TIMEOUT 900)

  ament_add_pytest_test(t01_expansion test/test_t01_expansion.py TIMEOUT 600)
  ament_add_pytest_test(t03_tree test/test_t03_tree.py TIMEOUT 600)
  ament_add_pytest_test(t06_arg_validation test/test_t06_arg_validation.py TIMEOUT 600)
  ament_add_pytest_test(t12_datums_schema test/test_t12_datums_schema.py TIMEOUT 600)
  ament_add_pytest_test(t13_datum_references test/test_t13_datum_references.py TIMEOUT 600)
  ament_add_pytest_test(t14_ros2_control_mock test/test_t14_ros2_control_mock.py TIMEOUT 600)
  ament_add_pytest_test(t15_ros2_control_gazebo test/test_t15_ros2_control_gazebo.py TIMEOUT 600)
  ament_add_pytest_test(t15_finger_mimic_multiplier test/test_t15_finger_mimic_multiplier.py TIMEOUT 600)
  ament_add_pytest_test(t16_ros2_control_real test/test_t16_ros2_control_real.py TIMEOUT 600)
  # simulator dynamics profile (config/sim_dynamics/openarm_v2.yaml): schema, a source for every value, the pins, the joint list
  ament_add_pytest_test(sim_dynamics_profile test/test_sim_dynamics_profile.py TIMEOUT 600)
  # the expansion: <dynamics> on all 16 arm and gripper joints of the simulated model, none in the real profile, nothing else moved
  ament_add_pytest_test(sim_dynamics_urdf test/test_sim_dynamics_urdf.py TIMEOUT 600)
  ament_add_pytest_test(t04_frame_contract test/test_t04_frame_contract.py TIMEOUT 600)
  # The combined robot IS option B's upper body plus the composition: every frame at the same pose, the rev19.0 numbers.
  ament_add_pytest_test(t04_optb_composition test/test_t04_optb_composition.py TIMEOUT 600)
  ament_add_pytest_test(t11_planar test/test_t11_planar.py TIMEOUT 600)
  ament_add_pytest_test(s4_sensors test/test_s4_sensors.py TIMEOUT 600)
  ament_add_pytest_test(t05_shoulder_height test/test_t05_shoulder_height.py TIMEOUT 600)
  ament_add_pytest_test(urdf_utils test/test_urdf_utils.py TIMEOUT 600)
  ament_add_pytest_test(datums_revision test/test_datums_revision.py TIMEOUT 600)
  ament_add_pytest_test(lift_fixing_text test/test_lift_fixing_text.py TIMEOUT 60)
  ament_add_pytest_test(mass_export_source test/test_mass_export_source.py TIMEOUT 60)
  ament_add_pytest_test(actuator_ledger test/test_actuator_ledger.py TIMEOUT 600)
  ament_add_pytest_test(datums_report test/test_datums_report.py TIMEOUT 600)
  ament_add_pytest_test(w01_kdl_root_inertia test/test_w01_kdl_root_inertia.py TIMEOUT 600)
  ament_add_pytest_test(t02_check_urdf test/test_t02_check_urdf.py TIMEOUT 600)
  ament_add_pytest_test(t07_collision test/test_t07_collision.py TIMEOUT 600)
  ament_add_pytest_test(t09_inertia test/test_t09_inertia.py TIMEOUT 600)
  ament_add_pytest_test(t10_mass_report test/test_t10_mass_report.py TIMEOUT 600)
  ament_add_pytest_test(roadmap_masses test/test_roadmap_masses.py TIMEOUT 600)
  ament_add_pytest_test(t13_visualization_constants test/test_t13_visualization_constants.py TIMEOUT 600)
  ament_add_pytest_test(e01_envelope test/test_e01_envelope.py TIMEOUT 600)
  ament_add_pytest_test(c01_config_id test/test_c01_config_id.py TIMEOUT 600)
  ament_add_pytest_test(c02_upstream_pin test/test_c02_upstream_pin.py TIMEOUT 600)
  ament_add_pytest_test(c02_platform_pin test/test_c02_platform_pin.py TIMEOUT 120)
  ament_add_pytest_test(w02_planar_kdl test/test_w02_planar_kdl.py TIMEOUT 600)
  # the three placeholder boxes have one explicit colour in the URDF and in the SDF (P25)
  ament_add_pytest_test(p25_placeholder_material test/test_p25_placeholder_material.py TIMEOUT 600)
  ament_add_pytest_test(p28_lift_geometry test/test_p28_lift_geometry.py TIMEOUT 600)
  # P29: quantify placeholder lift-stage collision-box overlap; not mechanical clearance.
  ament_add_pytest_test(p29_lift_stage_envelope test/test_p29_lift_stage_envelope.py TIMEOUT 600)
  # P-03 item 24 equipment: parents, static/moving side, poses = datums, stop-sensor heights, mass report (1 Nov 2026)
  ament_add_pytest_test(item24_equipment test/test_item24_equipment.py TIMEOUT 600)
  # tools/export_docs_model_values.py on the expanded URDF equals the docs model.js blocks the item 24 datums came from (0.5 mm)
  ament_add_pytest_test(item24_docs_model_values test/test_item24_docs_model_values.py TIMEOUT 600)
  ament_add_pytest_test(evidence_dir test/test_evidence_dir.py TIMEOUT 600)
  # the organisation's merge-marker scan, run on the tracked files with canaries (P27); needs git, skips outside a checkout
  ament_add_pytest_test(merge_markers test/test_merge_markers.py TIMEOUT 120)
  ament_add_pytest_test(build_evidence test/test_build_evidence.py TIMEOUT 600)
  # the detector of a gz server that outlives its launch, on decoy processes (P16); no ROS graph
  ament_add_pytest_test(launch_guard_gz_orphan test/test_launch_guard_gz_orphan.py TIMEOUT 120)
  ament_add_pytest_test(launch_guard_move_group_segv test/test_launch_guard_move_group_segv.py TIMEOUT 60)
endif()

ament_package()
