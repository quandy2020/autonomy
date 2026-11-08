# Helper: define an Argus OBJECT module and public alias.
#
#   argus_add_module(argus_common
#     DIRS argus/common
#     DEPS Eigen3::Eigen
#   )

function(argus_add_module NAME)
  cmake_parse_arguments(ARG "" "" "DIRS;DEPS;DEFINES" ${ARGN})
  if(NOT ARG_DIRS)
    message(FATAL_ERROR "argus_add_module(${NAME}): DIRS required")
  endif()

  set(_srcs "")
  set(_hdrs "")
  foreach(_dir IN LISTS ARG_DIRS)
    file(GLOB_RECURSE _d_srcs
      "${PROJECT_SOURCE_DIR}/${_dir}/*.cpp"
      "${PROJECT_SOURCE_DIR}/${_dir}/*.cc")
    file(GLOB_RECURSE _d_hdrs
      "${PROJECT_SOURCE_DIR}/${_dir}/*.hpp"
      "${PROJECT_SOURCE_DIR}/${_dir}/*.h")
    list(APPEND _srcs ${_d_srcs})
    list(APPEND _hdrs ${_d_hdrs})
  endforeach()

  if(_srcs)
    add_library(${NAME} OBJECT ${_srcs} ${_hdrs})
  else()
    # Header-only module: INTERFACE + empty OBJECT for uniform linking.
    add_library(${NAME} INTERFACE)
    if(_hdrs)
      target_sources(${NAME} INTERFACE ${_hdrs})
    endif()
  endif()

  if(TARGET ${NAME})
    get_target_property(_type ${NAME} TYPE)
    if(_type STREQUAL "INTERFACE_LIBRARY")
      target_include_directories(${NAME} INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS}
      )
      if(ARG_DEPS)
        target_link_libraries(${NAME} INTERFACE ${ARG_DEPS})
      endif()
      if(ARG_DEFINES)
        target_compile_definitions(${NAME} INTERFACE ${ARG_DEFINES})
      endif()
    else()
      set_target_properties(${NAME} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${ARGUS_CXX_FLAGS}"
      )
      target_include_directories(${NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS}
      )
      if(ARG_DEPS)
        target_link_libraries(${NAME} PUBLIC ${ARG_DEPS})
      endif()
      if(ARG_DEFINES)
        target_compile_definitions(${NAME} PUBLIC ${ARG_DEFINES})
      endif()
    endif()
  endif()

  add_library(Argus::${NAME} ALIAS ${NAME})
  set(ARGUS_MODULE_TARGETS ${ARGUS_MODULE_TARGETS} ${NAME} PARENT_SCOPE)
  get_target_property(_argus_mod_type ${NAME} TYPE)
  if(_argus_mod_type STREQUAL "OBJECT_LIBRARY")
    set(ARGUS_MODULE_OBJECTS
      ${ARGUS_MODULE_OBJECTS} $<TARGET_OBJECTS:${NAME}> PARENT_SCOPE)
  endif()
endfunction()
