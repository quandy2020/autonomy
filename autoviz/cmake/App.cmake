# =============================================================================
# Main autoviz executable and bundled sample tools
# =============================================================================

add_executable(autoviz
  ${AUTOVIZ_SOURCES}
  ${AUTOVIZ_HEADERS}
  ${AUTOVIZ_RECORDER_SOURCES}
  ${AUTOVIZ_ROOT}/resources/autoviz.qrc)

target_include_directories(autoviz PRIVATE ${AUTOVIZ_ROOT} ${AUTOVIZ_DEPS_ROOT})
add_dependencies(autoviz automsgs)

target_link_libraries(autoviz PRIVATE
  automsgs autolink yaml-cpp
  Qt6::Core Qt6::Gui Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets
  Qt6::Xml Qt6::Svg Qt6::Network
  glog protobuf::libprotobuf)

if(UNIX OR APPLE)
  target_link_libraries(autoviz PRIVATE ${CMAKE_DL_LIBS})
endif()
if(WIN32)
  target_link_libraries(autoviz PRIVATE kernel32)
endif()

if(AUTOVIZ_USE_QML_VEHICLE AND _AUTOVIZ_HAS_QML)
  add_subdirectory(${AUTOVIZ_ROOT}/qml ${CMAKE_BINARY_DIR}/qml)
  target_compile_definitions(autoviz PRIVATE AUTOVIZ_USE_QML_DRONE)
  target_link_libraries(autoviz PRIVATE
    autoviz_vehicle_qml autoviz_vehicle_qmlplugin autoviz_vehicle_qmlplugin_init
    Qt6::QuickWidgets)
endif()

include(OgreBackend)
autoviz_apply_ogre_backend(autoviz)

target_compile_features(autoviz PRIVATE cxx_std_17)
set_target_properties(autoviz PROPERTIES
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH "\$ORIGIN/../lib:${CMAKE_INSTALL_PREFIX}/lib"
  BUILD_RPATH "\$ORIGIN:\$ORIGIN/../lib:${CMAKE_BINARY_DIR}/lib")

if(APPLE)
  set_target_properties(autoviz PROPERTIES
    INSTALL_RPATH "@loader_path/../lib;@loader_path/../lib/autonomy"
    BUILD_RPATH "@loader_path;@loader_path/../lib")
endif()

# Sample: BICMap message publisher (dev / demo)
add_executable(autoviz_bicmap_publisher
  ${AUTOVIZ_ROOT}/tools/bicmap_example_publisher.cpp)
add_dependencies(autoviz_bicmap_publisher automsgs)
target_include_directories(autoviz_bicmap_publisher PRIVATE ${AUTOVIZ_ROOT} ${AUTOVIZ_DEPS_ROOT})
target_link_libraries(autoviz_bicmap_publisher PRIVATE
  automsgs autolink gflags glog protobuf::libprotobuf)
target_compile_features(autoviz_bicmap_publisher PRIVATE cxx_std_17)
