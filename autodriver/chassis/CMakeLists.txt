file(GLOB _chassis_headers CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${_chassis_headers}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/chassis")
install(FILES stub/driver.hpp
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/chassis/stub")

target_sources(autodriver PRIVATE
  chassis_manager.cpp
  chassis_component.cpp
  locomotion_model.cpp
  capability.cpp
  operational_mode.cpp
  safety_gate.cpp
  tool_command.cpp
  stub/driver.cpp)

add_subdirectory(jetauto)
add_subdirectory(l1w)
