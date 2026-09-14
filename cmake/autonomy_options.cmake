# Domain selection and dependency-graph validation.

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
