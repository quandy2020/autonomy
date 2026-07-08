# Taskflow: prefer vendored 3rdparty/taskflow, fall back to system install.
set(AUTONOMY_TASKFLOW_ROOT "${PROJECT_SOURCE_DIR}/3rdparty/taskflow")

if(EXISTS "${AUTONOMY_TASKFLOW_ROOT}/CMakeLists.txt")
  message(STATUS "Using vendored Taskflow from ${AUTONOMY_TASKFLOW_ROOT}")
  set(TF_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(TF_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(TF_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
  set(TF_BUILD_PROFILER OFF CACHE BOOL "" FORCE)
  set(TF_BUILD_CUDA OFF CACHE BOOL "" FORCE)
  add_subdirectory("${AUTONOMY_TASKFLOW_ROOT}" "${CMAKE_BINARY_DIR}/3rdparty/taskflow"
    EXCLUDE_FROM_ALL)
elseif(NOT TARGET Taskflow::Taskflow)
  find_package(Taskflow CONFIG REQUIRED)
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
