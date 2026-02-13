# Component demo: common_component_example (library + DAG / launch)

add_library(autonomy.examples.basic.common_component_example SHARED
  common_component_example.cpp ${BASIC_EXAMPLES_PB_CC})
add_dependencies(autonomy.examples.basic.common_component_example basic_examples_protos)

# proto 生成在 examples/basic/proto 下，这里是其父目录
target_include_directories(autonomy.examples.basic.common_component_example
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/..)

target_link_libraries(autonomy.examples.basic.common_component_example ${BASIC_DEPS})

# 输出到 bin/autonomy/examples/basic/component 下，库名为 libcommon_component_example.so
set_target_properties(autonomy.examples.basic.common_component_example PROPERTIES
  OUTPUT_NAME "common_component_example"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/autonomy/examples/basic/component")

# 将 DAG / launch 拷贝到构建目录，方便直接运行
file(COPY common.dag common.launch
  DESTINATION ${CMAKE_BINARY_DIR}/share/autonomy/examples/basic/component)

install(FILES common.dag common.launch
  DESTINATION share/autonomy/examples/basic/component)

install(TARGETS autonomy.examples.basic.common_component_example
  LIBRARY DESTINATION lib)

