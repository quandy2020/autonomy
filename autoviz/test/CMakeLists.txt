if(AUTOVIZ_BUILD_CORE_TESTS)
  find_package(Threads REQUIRED)
  add_executable(autoviz_interaction_test
    core/interaction_test.cpp
    ../autoviz/common/interaction/message_queue.cpp)
  target_link_libraries(autoviz_interaction_test PRIVATE Threads::Threads)
  target_include_directories(autoviz_interaction_test
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_compile_features(autoviz_interaction_test PRIVATE cxx_std_17)
  add_test(NAME autoviz_interaction_test COMMAND autoviz_interaction_test)
  set_tests_properties(autoviz_interaction_test PROPERTIES TIMEOUT 30)

  # Headless Qt regression for the session mosaic (MainPanelState) restore.
  add_executable(autoviz_mosaic_restore_test
    ui/mosaic_restore_test.cpp)
  target_include_directories(autoviz_mosaic_restore_test
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_compile_features(autoviz_mosaic_restore_test PRIVATE cxx_std_17)
  target_link_libraries(autoviz_mosaic_restore_test
    PRIVATE autoviz Qt6::Widgets)
  add_test(NAME autoviz_mosaic_restore_test
    COMMAND autoviz_mosaic_restore_test)
  set_tests_properties(autoviz_mosaic_restore_test PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    TIMEOUT 30)

  # Session (.autoviz) round-trip: globals, display tree, views, tool props.
  add_executable(autoviz_session_config_test
    core/session_config_test.cpp)
  target_include_directories(autoviz_session_config_test
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_compile_features(autoviz_session_config_test PRIVATE cxx_std_17)
  target_link_libraries(autoviz_session_config_test
    PRIVATE autoviz Qt6::Core Qt6::Gui)
  add_test(NAME autoviz_session_config_test
    COMMAND autoviz_session_config_test)
  set_tests_properties(autoviz_session_config_test PROPERTIES TIMEOUT 30)

  # Channel payload framing: MessageHeader stripping + fallbacks.
  add_executable(autoviz_channel_payload_test
    core/channel_payload_test.cpp)
  target_include_directories(autoviz_channel_payload_test
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_compile_features(autoviz_channel_payload_test PRIVATE cxx_std_17)
  target_link_libraries(autoviz_channel_payload_test PRIVATE autolink)
  add_test(NAME autoviz_channel_payload_test
    COMMAND autoviz_channel_payload_test)
  set_tests_properties(autoviz_channel_payload_test PROPERTIES TIMEOUT 30)

  # WebSocket binary frame: opcode + channel id + timestamp layout.
  add_executable(autoviz_ws_frame_test core/ws_frame_test.cpp)
  target_include_directories(autoviz_ws_frame_test
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_compile_features(autoviz_ws_frame_test PRIVATE cxx_std_17)
  add_test(NAME autoviz_ws_frame_test COMMAND autoviz_ws_frame_test)
  set_tests_properties(autoviz_ws_frame_test PROPERTIES TIMEOUT 30)

  # Display factory: every registered type must instantiate.
  add_executable(autoviz_display_registry_test
    core/display_registry_test.cpp)
  target_include_directories(autoviz_display_registry_test
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_compile_features(autoviz_display_registry_test PRIVATE cxx_std_17)
  target_link_libraries(autoviz_display_registry_test
    PRIVATE autoviz Qt6::Widgets)
  add_test(NAME autoviz_display_registry_test
    COMMAND autoviz_display_registry_test)
  set_tests_properties(autoviz_display_registry_test PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    TIMEOUT 30)

  # TF buffer: insert, chain lookup, revision, clear.
  add_executable(autoviz_tf_buffer_test core/tf_buffer_test.cpp)
  target_include_directories(autoviz_tf_buffer_test
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_compile_features(autoviz_tf_buffer_test PRIVATE cxx_std_17)
  target_link_libraries(autoviz_tf_buffer_test PRIVATE autoviz automsgs)
  add_test(NAME autoviz_tf_buffer_test COMMAND autoviz_tf_buffer_test)
  set_tests_properties(autoviz_tf_buffer_test PROPERTIES TIMEOUT 30)

  # WebSocket server: handshake, subscribe, binary broadcast (loopback).
  add_executable(autoviz_ws_server_test core/ws_server_test.cpp)
  target_include_directories(autoviz_ws_server_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
  target_compile_features(autoviz_ws_server_test PRIVATE cxx_std_17)
  target_link_libraries(autoviz_ws_server_test PRIVATE autoviz)
  add_test(NAME autoviz_ws_server_test COMMAND autoviz_ws_server_test)
  set_tests_properties(autoviz_ws_server_test PROPERTIES TIMEOUT 30)
endif()


if(AUTOVIZ_BUILD_WIDGET_TESTS)
find_package(Qt6 REQUIRED COMPONENTS Test)

function(autoviz_add_widget_test target source)
  add_executable(${target} ${source})
  target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_link_libraries(${target} PRIVATE autoviz Qt6::Test Qt6::Widgets)
  add_test(NAME ${target} COMMAND ${target} -o -,txt)
  set_tests_properties(${target} PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    TIMEOUT 30)
endfunction()

autoviz_add_widget_test(
  autoviz_widget_data_test widget/widget_data_test.cpp)
autoviz_add_widget_test(
  autoviz_widget_capability_closure_test widget/widget_capability_closure_test.cpp)
autoviz_add_widget_test(
  autoviz_widget_composite_test widget/widget_composite_test.cpp)
autoviz_add_widget_test(
  autoviz_widget_extended_components_test widget/widget_extended_components_test.cpp)
autoviz_add_widget_test(
  autoviz_widget_components_test widget/widget_components_test.cpp)
autoviz_add_widget_test(
  autoviz_widget_foundation_test widget/widget_foundation_test.cpp)
autoviz_add_widget_test(
  autoviz_widget_input_test widget/widget_input_test.cpp)
autoviz_add_widget_test(
  autoviz_widget_segmented_control_test widget/widget_segmented_control_test.cpp)
autoviz_add_widget_test(
  autoviz_widget_lifecycle_test widget/widget_lifecycle_test.cpp)
autoviz_add_widget_test(
  autoviz_widget_navigation_test widget/widget_navigation_test.cpp)
autoviz_add_widget_test(
  autoviz_widget_rtl_navigation_test widget/widget_rtl_navigation_test.cpp)
autoviz_add_widget_test(
  autoviz_widget_table_test widget/widget_table_test.cpp)
endif()  # AUTOVIZ_BUILD_WIDGET_TESTS
