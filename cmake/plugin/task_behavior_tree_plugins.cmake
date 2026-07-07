# Task-scoped BT plugins: autonomy/task/apps/<task>/plugins/<category>/<stem>.cpp
# Target: autonomy_task_<task>_<category>_<stem>

function(_autonomy_task_bt_plugin_target_name SRC_FILE OUT_VAR)
  set(_re "autonomy/task/apps/([^/]+)/plugins/([^/]+)/([^/]+)\\.cpp$")
  if(NOT "${SRC_FILE}" MATCHES "${_re}")
    message(FATAL_ERROR "Task BT plugin path not under apps/<task>/plugins/: ${SRC_FILE}")
  endif()
  set(${OUT_VAR}
    "autonomy_task_${CMAKE_MATCH_1}_${CMAKE_MATCH_2}_${CMAKE_MATCH_3}"
    PARENT_SCOPE)
endfunction()

function(_autonomy_configure_task_bt_plugin_target _target AUTONOMY_LIB)
  add_dependencies(${_target} ${AUTONOMY_LIB})
  target_compile_definitions(${_target} PRIVATE BT_PLUGIN_EXPORT)
  target_link_libraries(${_target} PRIVATE ${AUTONOMY_LIB})
  autonomy_link_behaviortree_cpp(${_target} PRIVATE)
  target_include_directories(${_target} PRIVATE
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_BINARY_DIR}")
  set_target_properties(${_target} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
    BUILD_RPATH "\$ORIGIN"
    SKIP_INSTALL_RPATH TRUE
  )
  install(TARGETS ${_target} LIBRARY DESTINATION lib)
endfunction()

function(autonomy_add_task_behavior_tree_plugins AUTONOMY_LIB)
  file(GLOB_RECURSE _task_bt_srcs
    "${PROJECT_SOURCE_DIR}/autonomy/task/apps/*/plugins/*/*.cpp")
  set(_task_bt_targets "")

  foreach(_src ${_task_bt_srcs})
    _autonomy_task_bt_plugin_target_name("${_src}" _target)
    add_library(${_target} SHARED "${_src}")
    _autonomy_configure_task_bt_plugin_target(${_target} ${AUTONOMY_LIB})
    list(APPEND _task_bt_targets ${_target})
  endforeach()

  if(_task_bt_targets)
    list(LENGTH _task_bt_targets _task_bt_count)
    add_custom_target(task_behavior_tree_plugins DEPENDS ${_task_bt_targets})
    message(STATUS "Task BT plugins: ${_task_bt_count} plugin(s) under "
      "autonomy/task/apps/*/plugins/")
  else()
    message(STATUS "Task BT plugins: none found under autonomy/task/apps/*/plugins/")
  endif()

  set(TASK_BEHAVIOR_TREE_PLUGIN_TARGETS "${_task_bt_targets}" PARENT_SCOPE)
endfunction()
