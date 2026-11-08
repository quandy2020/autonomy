# apps/benchmark — trajectory ATE / report generation

argus_binary(argus_benchmark
  SRCS main.cpp report_generator.cpp)

# Backward-compatible name used by docs / scripts.
add_executable(eval_pose_traj main.cpp report_generator.cpp)
target_link_libraries(eval_pose_traj PUBLIC ${PROJECT_NAME})
set_target_properties(eval_pose_traj PROPERTIES
  COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${ARGUS_CXX_FLAGS}")
