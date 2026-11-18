# Tests linked to actual native planning/task libraries and generated messages.
# Independent from BUILD_TEST so unrelated optional domains are not reconfigured.
if(AUTONOMY_BUILD_EXPLORATION_TESTS)
  if(NOT TARGET autonomy_planning OR NOT TARGET autonomy_task)
    message(FATAL_ERROR "Exploration integration tests require planning and task")
  endif()
  add_executable(autonomy_native_vista_test
    autonomy/planning/planner/vista/native_vista_test.cpp)
  target_link_libraries(autonomy_native_vista_test PRIVATE
    autonomy_planning autonomy_task autonomy_navigation_support)
  add_test(NAME autonomy.native.vista COMMAND autonomy_native_vista_test)
  set_tests_properties(autonomy.native.vista PROPERTIES TIMEOUT 30)

  add_executable(autonomy_vista_server_test
    autonomy/planning/planner/vista/vista_server_test.cpp)
  target_link_libraries(autonomy_vista_server_test PRIVATE
    autonomy_planning autonomy_task autonomy_navigation_support)
  add_test(NAME autonomy.planning.vista_server COMMAND autonomy_vista_server_test)
  set_tests_properties(autonomy.planning.vista_server PROPERTIES TIMEOUT 30)

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

  # Run coverage, session, phase and BT checks against the real domain libraries.
  # Test sources are excluded from the task library's recursive source collection.
  foreach(suite IN ITEMS multisensor_exploration exploration_session exploration_phase native_exploration_bt navigation_dispatcher exploration_owner)
    add_executable(autonomy_${suite}_test
      autonomy/task/exploration/${suite}_test.cpp)
    target_link_libraries(autonomy_${suite}_test PRIVATE
      autonomy_task autonomy_planning autonomy_navigation_support)
    add_test(NAME autonomy.exploration.${suite} COMMAND autonomy_${suite}_test)
    set_tests_properties(autonomy.exploration.${suite} PROPERTIES TIMEOUT 30)
  endforeach()
endif()
