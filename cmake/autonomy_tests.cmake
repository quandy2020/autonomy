# Super-project test collection (module libs GLOB themselves).

include_guard(GLOBAL)

function(autonomy_configure_tests)
  if(NOT DEFINED AUTONOMY_ENABLED_MODULES)
    message(FATAL_ERROR
      "autonomy_configure_tests: compute AUTONOMY_ENABLED_MODULES first")
  endif()
  set(_root "${PROJECT_SOURCE_DIR}/autonomy")
  set(AUTONOMY_TEST_MODULES "")

  foreach(_mod IN LISTS AUTONOMY_ENABLED_MODULES)
    if(NOT IS_DIRECTORY "${_root}/${_mod}")
      continue()
    endif()

    # Narrow GLOB: tests + helpers (filename patterns; recursive).
    file(GLOB_RECURSE _tests CONFIGURE_DEPENDS "${_root}/${_mod}/*_test.cpp")
    set(_module_helper_srcs "")
    set(_module_helper_hdrs "")

    file(GLOB_RECURSE _helper_srcs CONFIGURE_DEPENDS
      "${_root}/${_mod}/*fake*.cpp"
      "${_root}/${_mod}/*mock*.cpp"
      "${_root}/${_mod}/*test_helpers*.cpp")
    foreach(_f IN LISTS _helper_srcs)
      if(_f MATCHES "_test\\.cpp$")
        continue()
      endif()
      if(NOT _f MATCHES "/grid_map/grid_map_filters/mock_filter\\.cpp$")
        list(APPEND _module_helper_srcs "${_f}")
      endif()
    endforeach()

    file(GLOB_RECURSE _helper_hdrs CONFIGURE_DEPENDS
      "${_root}/${_mod}/*fake*.hpp"
      "${_root}/${_mod}/*mock*.hpp"
      "${_root}/${_mod}/*test_helpers*.hpp")
    foreach(_f IN LISTS _helper_hdrs)
      if(NOT _f MATCHES "/grid_map/grid_map_filters/mock_filter\\.hpp$")
        list(APPEND _module_helper_hdrs "${_f}")
      endif()
    endforeach()

    if(_mod STREQUAL "common")
      if(NOT AUTONOMY_BUILD_COMMON_OSQP)
        list(REMOVE_ITEM _tests
          "${_root}/common/math/mpc_osqp_test.cpp")
      endif()
      if(NOT Ipopt_FOUND)
        list(FILTER _tests EXCLUDE REGEX "/optimization/(ipopt|test)/")
      endif()
    elseif(_mod STREQUAL "visualization")
      list(REMOVE_ITEM _tests
        "${_root}/visualization/fakedata_test.cpp")
    elseif(_mod STREQUAL "perception")
      list(REMOVE_ITEM _tests
        "${_root}/perception/base/base_component_test.cpp")
    endif()

    list(REMOVE_DUPLICATES _tests)
    list(REMOVE_DUPLICATES _module_helper_srcs)
    list(REMOVE_DUPLICATES _module_helper_hdrs)
    list(APPEND AUTONOMY_TEST_MODULES "${_mod}")

    set(AUTONOMY_TESTS_${_mod} "${_tests}" PARENT_SCOPE)
    set(AUTONOMY_TEST_HELPER_SRCS_${_mod} "${_module_helper_srcs}" PARENT_SCOPE)
    set(AUTONOMY_TEST_HELPER_HDRS_${_mod} "${_module_helper_hdrs}" PARENT_SCOPE)
  endforeach()

  set(AUTONOMY_TEST_MODULES "${AUTONOMY_TEST_MODULES}" PARENT_SCOPE)
endfunction()
