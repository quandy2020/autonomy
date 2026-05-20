# Build BehaviorTree.CPP plugins as separate shared libraries (one .so per node).
# Names must match config/tasks/tasks.lua plugin_lib_names entries.

function(_autonomy_bt_plugin_target_name SRC_FILE OUT_VAR)
  get_filename_component(_stem "${SRC_FILE}" NAME_WE)
  get_filename_component(_parent "${SRC_FILE}" DIRECTORY)
  get_filename_component(_category "${_parent}" NAME)
  set(${OUT_VAR} "autonomy_behavior_tree_${_category}_${_stem}" PARENT_SCOPE)
endfunction()

function(autonomy_add_behavior_tree_plugins AUTONOMY_LIB)
  file(GLOB_RECURSE _bt_plugin_srcs
    "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/*.cpp")
  set(_bt_plugin_targets "")

  foreach(_src ${_bt_plugin_srcs})
    _autonomy_bt_plugin_target_name("${_src}" _target)
    add_library(${_target} SHARED "${_src}")
    add_dependencies(${_target} ${AUTONOMY_LIB})
    # Required by BehaviorTree.CPP: without BT_PLUGIN_EXPORT, BT_REGISTER_NODES
    # emits a static BT_RegisterNodesFromPlugin (invisible to registerFromPlugin).
    target_compile_definitions(${_target} PRIVATE BT_PLUGIN_EXPORT)
    target_link_libraries(${_target} PRIVATE ${AUTONOMY_LIB})
    if(TARGET BT::behaviortree_cpp)
      target_link_libraries(${_target} PRIVATE BT::behaviortree_cpp)
    elseif(TARGET behaviortree_cpp::behaviortree_cpp)
      target_link_libraries(${_target} PRIVATE behaviortree_cpp::behaviortree_cpp)
    endif()
    set_target_properties(${_target} PROPERTIES
      CXX_STANDARD 17
      CXX_STANDARD_REQUIRED ON
      POSITION_INDEPENDENT_CODE ON
      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
      BUILD_RPATH "\$ORIGIN"
      INSTALL_RPATH "\$ORIGIN"
    )
    install(TARGETS ${_target} LIBRARY DESTINATION lib)
    list(APPEND _bt_plugin_targets ${_target})
  endforeach()

  add_custom_target(behavior_tree_plugins DEPENDS ${_bt_plugin_targets})
  set(BEHAVIOR_TREE_PLUGIN_TARGETS "${_bt_plugin_targets}" PARENT_SCOPE)
endfunction()
