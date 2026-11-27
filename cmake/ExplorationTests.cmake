# Tests linked to actual native planning/task libraries and generated messages.
# Independent from BUILD_TEST so unrelated optional domains are not reconfigured.
if(AUTONOMY_BUILD_EXPLORATION_TESTS)
  set(AUTONOMY_DEFM_SAND_TEST_MODEL "" CACHE FILEPATH "Actual trained token-condition denoising graph")
  set(AUTONOMY_DEFM_SAND_TEST_FIXTURE "" CACHE FILEPATH "Actual held-out expert/native-step fixture")
  if(AUTONOMY_DEFM_SAND_TEST_MODEL AND AUTONOMY_DEFM_SAND_TEST_FIXTURE)
    add_executable(autonomy_native_defm_head_test
      autonomy/control/controller/policy_controller/sand_planner/native_defm_head_test.cpp)
    target_link_libraries(autonomy_native_defm_head_test PRIVATE autonomy_depth_features autonomy_control)
    add_test(NAME autonomy.control.native_defm_head COMMAND autonomy_native_defm_head_test
      "${AUTONOMY_DEFM_SAND_TEST_MODEL}" "${AUTONOMY_DEFM_TEST_MODEL}" "${AUTONOMY_DEFM_SAND_TEST_FIXTURE}")
    set_tests_properties(autonomy.control.native_defm_head PROPERTIES TIMEOUT 90)
  endif()
  add_executable(autonomy_measured_coverage_map_test autonomy/task/exploration/measured_coverage_map_test.cpp)
  target_link_libraries(autonomy_measured_coverage_map_test PRIVATE autonomy_task)
  add_test(NAME autonomy.exploration.measured_coverage_map COMMAND autonomy_measured_coverage_map_test)
  set_tests_properties(autonomy.exploration.measured_coverage_map PROPERTIES TIMEOUT 30)
  add_executable(autonomy_rolling_viewpoint_field_test
    autonomy/task/exploration/rolling_viewpoint_field_test.cpp)
  target_link_libraries(autonomy_rolling_viewpoint_field_test PRIVATE autonomy_task)
  add_test(NAME autonomy.exploration.rolling_viewpoint_field
    COMMAND autonomy_rolling_viewpoint_field_test)
  set_tests_properties(autonomy.exploration.rolling_viewpoint_field PROPERTIES TIMEOUT 30)
  add_executable(autonomy_native_controller_io_test autonomy/control/native_controller_io_test.cpp)
  target_link_libraries(autonomy_native_controller_io_test PRIVATE autonomy_control)
  add_test(NAME autonomy.control.native_controller_io COMMAND autonomy_native_controller_io_test)
  set_tests_properties(autonomy.control.native_controller_io PROPERTIES TIMEOUT 45)
  # Production Atlas bridge is optional; this regression retains native epoch
  # nanoseconds rather than weakening acquisition association with a tolerance.
  if(TARGET atlas_automsgs_bridge)
    add_executable(autonomy_native_observation_mapper_test
      autonomy/task/exploration/native_observation_mapper_test.cpp)
    target_link_libraries(autonomy_native_observation_mapper_test PRIVATE autonomy_task)
    add_test(NAME autonomy.exploration.native_observation_mapper COMMAND autonomy_native_observation_mapper_test)
    set_tests_properties(autonomy.exploration.native_observation_mapper PROPERTIES TIMEOUT 45)
    add_executable(autonomy_native_rgbd_platform_loader_test
      autonomy/task/exploration/native_rgbd_platform_loader_test.cpp)
    target_link_libraries(autonomy_native_rgbd_platform_loader_test PRIVATE autonomy_task)
    add_test(NAME autonomy.exploration.native_rgbd_platform_loader
      COMMAND autonomy_native_rgbd_platform_loader_test)
    set_tests_properties(autonomy.exploration.native_rgbd_platform_loader
      PROPERTIES TIMEOUT 30)
    add_executable(autonomy_atlas_acquisition_stamp_test
      autonomy/localization/atlas/tests/native_acquisition_stamp_test.cpp)
    target_link_libraries(autonomy_atlas_acquisition_stamp_test PRIVATE atlas_automsgs_bridge)
    add_test(NAME autonomy.localization.atlas_acquisition_stamp COMMAND autonomy_atlas_acquisition_stamp_test)
    set_tests_properties(autonomy.localization.atlas_acquisition_stamp PROPERTIES TIMEOUT 30)
  endif()
  # Real frozen weights and official preprocessing parity; missing artifacts fail.
  if(NOT TARGET autonomy_common)
    message(FATAL_ERROR "DeFM native tests require autonomy_common")
  endif()
  get_target_property(_defm_backend_definitions autonomy_common COMPILE_DEFINITIONS)
  if(NOT "AUTONOMY_HAS_ONNXRUNTIME" IN_LIST _defm_backend_definitions)
    message(FATAL_ERROR
      "DeFM native tests require the existing ONNX backend: BUILD_ONNXRUNTIME=ON and OnnxRuntime_ROOT pointing to a C++ SDK")
  endif()
  set(AUTONOMY_DEFM_TEST_MODEL "" CACHE FILEPATH "Exported real official DeFM graph")
  set(AUTONOMY_DEFM_TEST_FIXTURE "" CACHE FILEPATH "Independent official Torch parity fixture")
  add_executable(autonomy_defm_encoder_test
    autonomy/perception/models/defm/defm_encoder_test.cpp)
  target_link_libraries(autonomy_defm_encoder_test PRIVATE autonomy_depth_features autonomy_control)
  add_test(NAME autonomy.perception.defm_encoder COMMAND autonomy_defm_encoder_test
    "${AUTONOMY_DEFM_TEST_MODEL}" "${AUTONOMY_DEFM_TEST_FIXTURE}")
  set_tests_properties(autonomy.perception.defm_encoder PROPERTIES TIMEOUT 90)
  # Actual scheduler and parent/child native task admission, not a model scheduler.
  add_executable(autonomy_scheduler_motion_test
    autonomy/task/scheduler/scheduler_motion_test.cpp)
  target_link_libraries(autonomy_scheduler_motion_test PRIVATE autonomy_task autonomy_planning autonomy_common)
  add_test(NAME autonomy.task.scheduler_motion COMMAND autonomy_scheduler_motion_test)
  set_tests_properties(autonomy.task.scheduler_motion PROPERTIES TIMEOUT 45)
  # The exact retirement helper is also verified with the actual support target.
  add_executable(autonomy_reader_retirement_test
    autonomy/perception/observation/reader_retirement_test.cpp)
  target_link_libraries(autonomy_reader_retirement_test PRIVATE autonomy_common)
  add_test(NAME autonomy.exploration.reader_retirement COMMAND autonomy_reader_retirement_test)
  set_tests_properties(autonomy.exploration.reader_retirement PROPERTIES TIMEOUT 15)

  # Actual Node/Reader/Writer, generated messages and scheduler lifecycle.
  # Production sources belong to autonomy_common, including when optional
  # perception applications are disabled. Every consumer links that same implementation.
  add_executable(autonomy_native_observation_source_test
    autonomy/perception/observation/native_observation_source_test.cpp)
  target_link_libraries(autonomy_native_observation_source_test PRIVATE
    autonomy_common autolink automsgs)
  add_test(NAME autonomy.exploration.native_observation_source COMMAND autonomy_native_observation_source_test)
  set_tests_properties(autonomy.exploration.native_observation_source PROPERTIES TIMEOUT 60)

  add_executable(autonomy_native_atlas_localization_test
    autonomy/perception/observation/native_atlas_localization_test.cpp)
  target_link_libraries(autonomy_native_atlas_localization_test PRIVATE autonomy_common)
  add_test(NAME autonomy.exploration.native_atlas_localization COMMAND autonomy_native_atlas_localization_test)
  set_tests_properties(autonomy.exploration.native_atlas_localization PROPERTIES TIMEOUT 30)

  add_executable(autonomy_observation_ingress_test
    autonomy/perception/observation/observation_ingress_test.cpp)
  target_link_libraries(autonomy_observation_ingress_test PRIVATE autonomy_task autonomy_common)
  add_test(NAME autonomy.exploration.observation_ingress COMMAND autonomy_observation_ingress_test)
  set_tests_properties(autonomy.exploration.observation_ingress PROPERTIES TIMEOUT 30)
  if(NOT TARGET autonomy_planning OR NOT TARGET autonomy_task)
    message(FATAL_ERROR "Exploration integration tests require planning and task")
  endif()
  # Shared math used by both SanD sampling and timed-reference derivatives.
  add_executable(autonomy_shared_bspline_cache_test
    autonomy/common/math/bspline_cache_test.cpp)
  target_include_directories(autonomy_shared_bspline_cache_test PRIVATE ${PROJECT_SOURCE_DIR})
  target_compile_features(autonomy_shared_bspline_cache_test PRIVATE cxx_std_17)
  target_link_libraries(autonomy_shared_bspline_cache_test PRIVATE Threads::Threads)
  add_test(NAME autonomy.common.bspline_cache COMMAND autonomy_shared_bspline_cache_test)
  set_tests_properties(autonomy.common.bspline_cache PROPERTIES TIMEOUT 15)
  # Use actual generated automsgs and the production conversion implementation.
  add_executable(autonomy_native_sensor_messages_test
    autonomy/perception/observation/native_sensor_messages_test.cpp)
  target_link_libraries(autonomy_native_sensor_messages_test PRIVATE
    autonomy_common automsgs)
  add_test(NAME autonomy.exploration.native_sensor_messages COMMAND autonomy_native_sensor_messages_test)
  set_tests_properties(autonomy.exploration.native_sensor_messages PROPERTIES TIMEOUT 30)
  add_executable(autonomy_native_vista_test
    autonomy/planning/planner/vista/native_vista_test.cpp)
  target_link_libraries(autonomy_native_vista_test PRIVATE
    autonomy_planning autonomy_task autonomy_common)
  add_test(NAME autonomy.native.vista COMMAND autonomy_native_vista_test)
  set_tests_properties(autonomy.native.vista PROPERTIES TIMEOUT 30)

  add_executable(autonomy_vista_server_test
    autonomy/planning/planner/vista/vista_server_test.cpp)
  target_link_libraries(autonomy_vista_server_test PRIVATE
    autonomy_planning autonomy_task autonomy_common)
  add_test(NAME autonomy.planning.vista_server COMMAND autonomy_vista_server_test)
  set_tests_properties(autonomy.planning.vista_server PROPERTIES TIMEOUT 30)

  # Actual route service, separate native client process and bounded RPC lifetime.
  add_executable(autonomy_native_route_service_test
    autonomy/planning/planner/vista/native_route_service_test.cpp)
  target_link_libraries(autonomy_native_route_service_test PRIVATE
    autonomy_planning autonomy_task autonomy_common)
  add_test(NAME autonomy.planning.native_route_service COMMAND autonomy_native_route_service_test)
  set_tests_properties(autonomy.planning.native_route_service PROPERTIES TIMEOUT 60)

  # Real graph updater, skipped deltas, bounded spatial index and epoch contracts.
  add_executable(autonomy_vista_incremental_graph_test
    autonomy/planning/planner/vista/incremental_graph_test.cpp)
  target_link_libraries(autonomy_vista_incremental_graph_test PRIVATE
    autonomy_planning autonomy_task autonomy_common)
  add_test(NAME autonomy.planning.vista_incremental_graph COMMAND autonomy_vista_incremental_graph_test)
  set_tests_properties(autonomy.planning.vista_incremental_graph PROPERTIES TIMEOUT 30)

  # Exercise the real runner, typed host and node side effects, not a mock loop.
  add_executable(autonomy_bt_runner_test
    autonomy/task/behavior_tree/bt_runner_test.cpp)
  target_link_libraries(autonomy_bt_runner_test PRIVATE autonomy_task)
  add_test(NAME autonomy.task.bt_runner COMMAND autonomy_bt_runner_test)
  set_tests_properties(autonomy.task.bt_runner PROPERTIES TIMEOUT 30)

  # Native typed acceptance must never attribute old results to rejected goals.
  add_executable(autonomy_typed_task_test
    autonomy/task/common/typed_task_test.cpp)
  target_link_libraries(autonomy_typed_task_test PRIVATE autonomy_task)
  add_test(NAME autonomy.task.typed_task COMMAND autonomy_typed_task_test)
  set_tests_properties(autonomy.task.typed_task PROPERTIES TIMEOUT 30)

  # Exercise actual NavigationTask, identity admission and native worker nodes.
  add_executable(autonomy_navigation_task_test
    autonomy/task/navigation/navigation_task_test.cpp)
  target_link_libraries(autonomy_navigation_task_test PRIVATE autonomy_task)
  add_test(NAME autonomy.task.navigation COMMAND autonomy_navigation_task_test)
  set_tests_properties(autonomy.task.navigation PROPERTIES TIMEOUT 45)

  # Exact production wait used by the native action Client; promises remain real.
  add_executable(autonomy_result_response_wait_test
    autonomy/task/navigation/result_response_wait_test.cpp)
  target_include_directories(autonomy_result_response_wait_test PRIVATE
    ${PROJECT_SOURCE_DIR}/autolink)
  target_compile_features(autonomy_result_response_wait_test PRIVATE cxx_std_17)
  add_test(NAME autonomy.task.result_response_wait COMMAND autonomy_result_response_wait_test)
  set_tests_properties(autonomy.task.result_response_wait PROPERTIES TIMEOUT 15)

  # The exact production route helper; no middleware seam or endpoint-only hash.
  add_executable(autonomy_path_reference_test
    autonomy/task/navigation/path_reference_test.cpp)
  target_include_directories(autonomy_path_reference_test PRIVATE ${PROJECT_SOURCE_DIR})
  target_compile_features(autonomy_path_reference_test PRIVATE cxx_std_17)
  add_test(NAME autonomy.task.path_reference COMMAND autonomy_path_reference_test)
  set_tests_properties(autonomy.task.path_reference PROPERTIES TIMEOUT 15)

  # Real action sessions and NavigationTask, controlling only RPC ordering.
  add_executable(autonomy_motion_dispatch_test
    autonomy/task/navigation/motion_dispatch_test.cpp)
  target_link_libraries(autonomy_motion_dispatch_test PRIVATE autonomy_task)
  add_test(NAME autonomy.task.motion_dispatch COMMAND autonomy_motion_dispatch_test)
  set_tests_properties(autonomy.task.motion_dispatch PROPERTIES TIMEOUT 45)

  # Production gate races/lifetimes also run independently of controlled RPCs.
  add_executable(autonomy_motion_dispatch_gate_test
    autonomy/task/common/motion_dispatch_gate_test.cpp)
  target_link_libraries(autonomy_motion_dispatch_gate_test PRIVATE autonomy_task)
  add_test(NAME autonomy.task.motion_dispatch_gate COMMAND autonomy_motion_dispatch_gate_test)
  set_tests_properties(autonomy.task.motion_dispatch_gate PROPERTIES TIMEOUT 15)

  # Run coverage, session, phase and BT checks against the real domain libraries.
  # Test sources are excluded from the task library's recursive source collection.
  foreach(suite IN ITEMS acquisition_keypose_ledger momentum_tracker frontier_extractor surface_extractor frontier_queue region_manager visit_order_optimizer goal_pose_adapter route_lookahead_selector route_line_of_sight_view_producer multisensor_exploration exploration_session exploration_phase native_exploration_bt navigation_dispatcher exploration_owner hierarchical_coverage_strategy exploration_input_prefix exploration_loss_recovery exploration_observation_source native_session_factory)
    add_executable(autonomy_${suite}_test
      autonomy/task/exploration/${suite}_test.cpp)
    target_link_libraries(autonomy_${suite}_test PRIVATE
      autonomy_task autonomy_planning autonomy_common)
    add_test(NAME autonomy.exploration.${suite} COMMAND autonomy_${suite}_test)
    set_tests_properties(autonomy.exploration.${suite} PROPERTIES TIMEOUT 30)
  endforeach()
endif()
