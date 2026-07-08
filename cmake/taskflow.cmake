# Taskflow: vendored 3rdparty/taskflow -> system install -> FetchContent.
set(AUTONOMY_TASKFLOW_ROOT "${PROJECT_SOURCE_DIR}/3rdparty/taskflow")
set(AUTONOMY_TASKFLOW_VERSION "4.1.0")

macro(autonomy_configure_taskflow_build_options)
  set(TF_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(TF_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(TF_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
  set(TF_BUILD_PROFILER OFF CACHE BOOL "" FORCE)
  set(TF_BUILD_CUDA OFF CACHE BOOL "" FORCE)
endmacro()

if(EXISTS "${AUTONOMY_TASKFLOW_ROOT}/CMakeLists.txt")
  message(STATUS "Using vendored Taskflow from ${AUTONOMY_TASKFLOW_ROOT}")
  autonomy_configure_taskflow_build_options()
  add_subdirectory("${AUTONOMY_TASKFLOW_ROOT}" "${CMAKE_BINARY_DIR}/3rdparty/taskflow"
    EXCLUDE_FROM_ALL)
elseif(NOT TARGET Taskflow::Taskflow)
  find_package(Taskflow CONFIG QUIET
    HINTS
      "${CMAKE_INSTALL_PREFIX}"
      "/usr/local"
      "$ENV{HOME}/.local")

  if(NOT Taskflow_FOUND)
    include(FetchContent)
    message(STATUS "Taskflow not found locally; fetching v${AUTONOMY_TASKFLOW_VERSION} via FetchContent")
    autonomy_configure_taskflow_build_options()
    FetchContent_Declare(
      taskflow
      GIT_REPOSITORY https://github.com/taskflow/taskflow.git
      GIT_TAG "v${AUTONOMY_TASKFLOW_VERSION}"
      GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(taskflow)
  endif()
endif()

if(NOT TARGET Taskflow::Taskflow)
  message(FATAL_ERROR "Taskflow target not found after configure")
endif()

function(autonomy_link_taskflow target_name visibility)
  if(NOT AUTONOMY_TASK_SCHEDULER_SRCS)
    message(FATAL_ERROR "AUTONOMY_TASK_SCHEDULER_SRCS must be set before autonomy_link_taskflow")
  endif()

  add_library(autonomy_task_scheduler OBJECT ${AUTONOMY_TASK_SCHEDULER_SRCS})
  set_target_properties(autonomy_task_scheduler PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_compile_features(autonomy_task_scheduler PRIVATE cxx_std_20)
  target_link_libraries(autonomy_task_scheduler PUBLIC Taskflow::Taskflow)
  target_include_directories(autonomy_task_scheduler
    PUBLIC
      ${PROJECT_SOURCE_DIR}
      ${PROJECT_BINARY_DIR})

  # Scheduler headers include generated *.pb.h; without this, Ninja may compile
  # scheduler.cpp before protoc finishes (parallel build race).
  if(AUTONOMY_PROTO_GENERATED_HDRS)
    target_sources(autonomy_task_scheduler PRIVATE ${AUTONOMY_PROTO_GENERATED_HDRS})
    set_source_files_properties(${AUTONOMY_PROTO_GENERATED_HDRS} PROPERTIES GENERATED TRUE)
  endif()

  target_link_libraries(${target_name} ${visibility} Taskflow::Taskflow)
  target_sources(${target_name} PRIVATE $<TARGET_OBJECTS:autonomy_task_scheduler>)
endfunction()
