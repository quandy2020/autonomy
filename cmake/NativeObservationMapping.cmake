# Copyright 2026 The Openbot Authors.
# Native task integration reuses the actual enabled Atlas, ESDF and grid_map
# libraries. The existing Task/Planning CMake files remain untouched.
if(TARGET atlas_core AND TARGET autonomy_task)
  target_compile_definitions(autonomy_task PRIVATE AUTONOMY_TASK_HAS_NATIVE_ATLAS_MAPPING)
  target_link_libraries(autonomy_task PRIVATE atlas_core)
  if(TARGET autonomy.task)
    target_compile_definitions(autonomy.task PRIVATE AUTONOMY_TASK_HAS_NATIVE_ATLAS_MAPPING)
  endif()
endif()
