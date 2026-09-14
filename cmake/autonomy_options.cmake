# @file autonomy_options.cmake
# @brief Domain AUTONOMY_BUILD_* options, dependency graph, and enabled-module
#        list computation.
#
# @details @c AUTONOMY_MODULE_DEPENDENCIES_${mod} defines hard deps between
#          domains.

include_guard(GLOBAL)

set(AUTONOMY_MODULE_DEPENDENCIES_common "")
set(AUTONOMY_MODULE_DEPENDENCIES_transform common)
set(AUTONOMY_MODULE_DEPENDENCIES_map common transform)
set(AUTONOMY_MODULE_DEPENDENCIES_vehicle common)
set(AUTONOMY_MODULE_DEPENDENCIES_prediction common)
set(AUTONOMY_MODULE_DEPENDENCIES_control common transform map)
set(AUTONOMY_MODULE_DEPENDENCIES_planning common transform map)
set(AUTONOMY_MODULE_DEPENDENCIES_perception common transform map)
set(AUTONOMY_MODULE_DEPENDENCIES_localization common transform)
set(AUTONOMY_MODULE_DEPENDENCIES_sensor common control)
set(AUTONOMY_MODULE_DEPENDENCIES_task common transform map control)
set(AUTONOMY_MODULE_DEPENDENCIES_system common task)
set(AUTONOMY_MODULE_DEPENDENCIES_audio common)
set(AUTONOMY_MODULE_DEPENDENCIES_bridge common system task)
set(AUTONOMY_MODULE_DEPENDENCIES_visualization map)

# @brief Declare option(AUTONOMY_BUILD_*) for each entry in
#        @c AUTONOMY_MODULE_ORDER.
# @pre @c AUTONOMY_MODULE_ORDER must be set.
function(autonomy_declare_module_options)
  if(NOT DEFINED AUTONOMY_MODULE_ORDER)
    message(FATAL_ERROR
      "autonomy_declare_module_options: set AUTONOMY_MODULE_ORDER first")
  endif()

  foreach(_module IN LISTS AUTONOMY_MODULE_ORDER)
    string(TOUPPER "${_module}" _module_upper)
    option(AUTONOMY_BUILD_${_module_upper}
      "Build the autonomy ${_module} module" ON)
  endforeach()
endfunction()

# @brief Enabled domains must also enable every domain listed in
#        @c AUTONOMY_MODULE_DEPENDENCIES_*.
function(autonomy_validate_module_graph)
  if(NOT DEFINED AUTONOMY_MODULE_ORDER)
    message(FATAL_ERROR
      "autonomy_validate_module_graph: set AUTONOMY_MODULE_ORDER first")
  endif()

  foreach(_module IN LISTS AUTONOMY_MODULE_ORDER)
    string(TOUPPER "${_module}" _module_upper)
    if(NOT AUTONOMY_BUILD_${_module_upper})
      continue()
    endif()

    foreach(_dependency IN LISTS AUTONOMY_MODULE_DEPENDENCIES_${_module})
      string(TOUPPER "${_dependency}" _dependency_upper)
      if(NOT AUTONOMY_BUILD_${_dependency_upper})
        message(FATAL_ERROR
          "AUTONOMY_BUILD_${_module_upper}=ON requires "
          "AUTONOMY_BUILD_${_dependency_upper}=ON")
      endif()
    endforeach()
  endforeach()
endfunction()

# @brief Write the enabled-domain list from AUTONOMY_BUILD_* options.
# @param _out_variable Parent-scope output variable name
#        (e.g. AUTONOMY_ENABLED_MODULES).
function(autonomy_compute_enabled_modules _out_variable)
  if(NOT DEFINED AUTONOMY_MODULE_ORDER)
    message(FATAL_ERROR
      "autonomy_compute_enabled_modules: set AUTONOMY_MODULE_ORDER first")
  endif()

  set(_enabled_modules "")
  foreach(_module IN LISTS AUTONOMY_MODULE_ORDER)
    string(TOUPPER "${_module}" _module_upper)
    if(AUTONOMY_BUILD_${_module_upper})
      list(APPEND _enabled_modules "${_module}")
    endif()
  endforeach()
  set(${_out_variable} "${_enabled_modules}" PARENT_SCOPE)
endfunction()
