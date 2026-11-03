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
  autoviz_widget_input_test widget/widget_input_test.cpp)
autoviz_add_widget_test(
  autoviz_widget_lifecycle_test widget/widget_lifecycle_test.cpp)
