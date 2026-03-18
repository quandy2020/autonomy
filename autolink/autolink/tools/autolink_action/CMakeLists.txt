# C++ 工具：autolink_action（对应 ROS2-style action 命令行）
add_executable(autolink_action main.cpp)
target_include_directories(autolink_action PRIVATE BEFORE
  ${CMAKE_SOURCE_DIR}
)
target_link_libraries(autolink_action PRIVATE autolink)

install(TARGETS autolink_action
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
