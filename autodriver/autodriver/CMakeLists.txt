# Public headers. conf.hpp and version.json are generated in this binary dir.
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/autodriver"
  FILES_MATCHING PATTERN "*.hpp")

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/conf/conf.hpp"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/autodriver/conf")

if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/version.json")
  install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/version.json"
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/autodriver")
endif()
