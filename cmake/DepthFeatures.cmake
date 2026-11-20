# Frozen depth inference is a perception-owned library, also usable when the
# optional perception applications are disabled. Lightweight map/observation
# support keeps its existing dependency graph and contains no ML implementation.
if(TARGET autonomy_common)
  add_library(autonomy_depth_features STATIC
    autonomy/perception/models/defm/defm_encoder.cpp)
  add_library(autonomy::depth_features ALIAS autonomy_depth_features)
  target_link_libraries(autonomy_depth_features PUBLIC
    autonomy_common automsgs)
  target_include_directories(autonomy_depth_features PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>)
  target_compile_features(autonomy_depth_features PUBLIC cxx_std_20)
  set_target_properties(autonomy_depth_features PROPERTIES POSITION_INDEPENDENT_CODE ON)
  if(TARGET autonomy_control)
    target_link_libraries(autonomy_control PUBLIC autonomy_depth_features)
  endif()
endif()
