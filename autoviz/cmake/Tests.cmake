# =============================================================================
# Dev gtests (-DBUILD_AUTOVIZ_TESTS=ON; Ogre tests also need -DAUTOVIZ_USE_OGRE=ON)
# =============================================================================

enable_testing()
find_package(GTest QUIET)

# GUI tests need DISPLAY; default :1 for headless / Xvfb setups.
function(_autoviz_test_display _name)
  if(DEFINED ENV{DISPLAY})
    set(_d "$ENV{DISPLAY}")
  else()
    set(_d ":1")
  endif()
  set_tests_properties(${_name} PROPERTIES ENVIRONMENT "DISPLAY=${_d}")
endfunction()

function(_autoviz_ogre_gtest _name)
  add_executable(${_name} ${ARGN})
  add_dependencies(${_name} automsgs)
  target_include_directories(${_name} PRIVATE ${AUTOVIZ_ROOT} ${AUTOVIZ_DEPS_ROOT})
  target_compile_definitions(${_name} PRIVATE
    AUTOVIZ_USE_OGRE
    AUTOVIZ_OGRE_MEDIA_DIR="${AUTOVIZ_ROOT}/resources/ogre_media"
    AUTOVIZ_TEST_MESHES_DIR="${AUTOVIZ_ROOT}/tests/ogre_media_resources")
  if(_AUTOVIZ_OGRE_RVIZ_MEDIA)
    target_compile_definitions(${_name} PRIVATE AUTOVIZ_OGRE_RVIZ_MEDIA)
  endif()
  if(_AUTOVIZ_OGRE_PLUGIN_DIR)
    target_compile_definitions(${_name} PRIVATE AUTOVIZ_OGRE_PLUGIN_DIR="${_AUTOVIZ_OGRE_PLUGIN_DIR}")
  endif()
  if(_AUTOVIZ_HAS_ASSIMP)
    target_compile_definitions(${_name} PRIVATE AUTOVIZ_USE_ASSIMP)
  endif()
  target_include_directories(${_name} SYSTEM PRIVATE ${_AUTOVIZ_OGRE_INCLUDE_DIRS})
  target_link_libraries(${_name} PRIVATE
    automsgs GTest::gtest Qt6::Gui Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets
    Qt6::Network Eigen3::Eigen glog::glog protobuf::libprotobuf
    ${_AUTOVIZ_OGRE_TARGET} ${_AUTOVIZ_OGRE_OVERLAY_TARGET} ${CMAKE_DL_LIBS})
  if(_AUTOVIZ_HAS_ASSIMP)
    if(assimp_FOUND)
      target_link_libraries(${_name} PRIVATE assimp::assimp)
    else()
      target_include_directories(${_name} SYSTEM PRIVATE ${ASSIMP_INCLUDE_DIRS})
      target_link_libraries(${_name} PRIVATE ${ASSIMP_LIBRARIES})
    endif()
  endif()
  if(UNIX AND NOT APPLE)
    find_package(X11 QUIET)
    if(X11_FOUND)
      target_link_libraries(${_name} PRIVATE X11::X11 GL)
    else()
      target_link_libraries(${_name} PRIVATE X11 GL)
    endif()
  endif()
  target_compile_features(${_name} PRIVATE cxx_std_17)
  add_test(NAME ${_name} COMMAND ${_name})
  _autoviz_test_display(${_name})
endfunction()

function(_autoviz_display_gtest _name)
  add_executable(${_name} ${ARGN})
  add_dependencies(${_name} automsgs)
  target_include_directories(${_name} PRIVATE ${AUTOVIZ_ROOT} ${AUTOVIZ_DEPS_ROOT})
  target_link_libraries(${_name} PRIVATE
    automsgs autolink yaml-cpp GTest::gtest
    Qt6::Gui Qt6::Widgets Qt6::OpenGL Qt6::Network
    glog::glog protobuf::libprotobuf ${CMAKE_DL_LIBS})
  target_compile_features(${_name} PRIVATE cxx_std_17)
  add_test(NAME ${_name} COMMAND ${_name})
  _autoviz_test_display(${_name})
endfunction()

if(AUTOVIZ_USE_OGRE)
  add_executable(autoviz_ogre_pick_verify
    ${AUTOVIZ_ROOT}/tests/ogre_pick_verify.cpp
    ${AUTOVIZ_SRC_ROOT}/common/pick_registry.cpp
    ${AUTOVIZ_SRC_ROOT}/common/pick_handle.cpp)
  target_include_directories(autoviz_ogre_pick_verify PRIVATE ${AUTOVIZ_ROOT})
  target_link_libraries(autoviz_ogre_pick_verify PRIVATE Qt6::Gui)
  target_compile_features(autoviz_ogre_pick_verify PRIVATE cxx_std_17)

  if(GTest_FOUND)
    set(_gtest_main ${AUTOVIZ_ROOT}/tests/gtest_main.cpp)
    set(_ogre_env
      ${AUTOVIZ_ROOT}/tests/ogre_testing_environment.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/render_system.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/ogre_resource_config.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/ogre_material_manager.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/ogre_procedural_shape.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/ogre_logging.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/gpu_capabilities.cpp
      ${AUTOVIZ_SRC_ROOT}/display/primitive_mesh.cpp
      ${AUTOVIZ_SRC_ROOT}/display/obj_mesh.cpp)
    set(_mesh_loader ${_gtest_main} ${AUTOVIZ_ROOT}/tests/mesh_loader_test.cpp
      ${_ogre_env}
      ${AUTOVIZ_SRC_ROOT}/rendering/mesh_resource.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/ogre_mesh_loader.cpp)
    if(_AUTOVIZ_HAS_ASSIMP)
      list(APPEND _mesh_loader ${AUTOVIZ_SRC_ROOT}/rendering/mesh_loader_helpers/assimp_loader.cpp)
    endif()
    _autoviz_ogre_gtest(autoviz_mesh_loader_test ${_mesh_loader})
    _autoviz_ogre_gtest(autoviz_rendering_objects_test
      ${_gtest_main} ${AUTOVIZ_ROOT}/tests/rendering_objects_test.cpp ${_ogre_env}
      ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_line.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_shape.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_arrow.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_billboard_line.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_wrench_visual.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_screw_visual.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_mesh_shape.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_covariance_visual.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_point_cloud.cpp
      ${AUTOVIZ_SRC_ROOT}/rendering/objects/ogre_point_cloud_renderable.cpp)
  else()
    message(STATUS "GTest not found; skipping Ogre gtests")
  endif()
endif()

if(GTest_FOUND)
  # BICMap display integration test: compile display sources directly (no libautonomy).
  set(_bicmap_display_sources
    ${AUTOVIZ_SRC_ROOT}/display/display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/map_display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/path_display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/point_cloud2_display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/point_cloud_utils.cpp
    ${AUTOVIZ_SRC_ROOT}/display/marker_array_display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/marker_draw_utils.cpp
    ${AUTOVIZ_SRC_ROOT}/display/strata_poi_display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/strata_robot_display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/strata_semantic_zone_display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/strata_fov_display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/strata_road_graph_display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/strata_canvas_label_display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/strata_label_bubble_display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/strata_iot_bubble_display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/strata_robot3d_display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/strata_building_display.cpp
    ${AUTOVIZ_SRC_ROOT}/display/screen_projection.cpp
    ${AUTOVIZ_SRC_ROOT}/display/transform_utils.cpp
    ${AUTOVIZ_SRC_ROOT}/display/ogre_overlay_draw.cpp
    ${AUTOVIZ_SRC_ROOT}/display/ogre_label_draw.cpp
    ${AUTOVIZ_SRC_ROOT}/display/ogre_colored_points_draw.cpp
    ${AUTOVIZ_SRC_ROOT}/display/ogre_mesh_draw.cpp
    ${AUTOVIZ_SRC_ROOT}/display/primitive_mesh.cpp
    ${AUTOVIZ_SRC_ROOT}/display/obj_mesh.cpp
    ${AUTOVIZ_SRC_ROOT}/display/arrow_mesh_utils.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/scene_overlay.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/grid_renderer.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/text_raster_utils.cpp
    ${AUTOVIZ_SRC_ROOT}/rendering/point_cloud_style_utils.cpp
    ${AUTOVIZ_SRC_ROOT}/common/config.cpp
    ${AUTOVIZ_SRC_ROOT}/common/display_property.cpp
    ${AUTOVIZ_SRC_ROOT}/common/pick_registry.cpp
    ${AUTOVIZ_SRC_ROOT}/common/pick_handle.cpp
    ${AUTOVIZ_SRC_ROOT}/common/selection_handler.cpp
    ${AUTOVIZ_SRC_ROOT}/transform/buffer.cpp
    ${AUTOVIZ_SRC_ROOT}/transform/buffer_utils.cpp
    ${AUTOVIZ_SRC_ROOT}/transform/tf2/buffer_core.cpp
    ${AUTOVIZ_SRC_ROOT}/transform/tf2/cache.cpp
    ${AUTOVIZ_SRC_ROOT}/transform/tf2/static_cache.cpp
    ${AUTOVIZ_SRC_ROOT}/transform/tf2/time.cpp)
  _autoviz_display_gtest(autoviz_bicmap_examples_test
    ${AUTOVIZ_ROOT}/tests/bicmap_gtest_main.cpp
    ${AUTOVIZ_ROOT}/tests/bicmap_examples_test.cpp
    ${_bicmap_display_sources})

  _autoviz_display_gtest(tf_display_utils_test
    ${AUTOVIZ_ROOT}/tests/gtest_main.cpp
    ${AUTOVIZ_ROOT}/tests/tf_display_utils_test.cpp
    ${AUTOVIZ_SRC_ROOT}/display/tf_display_utils.cpp)
endif()
