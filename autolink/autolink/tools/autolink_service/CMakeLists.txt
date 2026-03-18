# C++ 工具：autolink_service（对应 Autolink tools service 命令）
add_executable(autolink_service main.cpp)
target_include_directories(autolink_service PRIVATE BEFORE
  ${CMAKE_SOURCE_DIR}
)
target_link_libraries(autolink_service PRIVATE autolink)

install(TARGETS autolink_service
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
