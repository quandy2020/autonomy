file(GLOB _arm_headers CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${_arm_headers}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/arm")
install(FILES stub/driver.hpp
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/arm/stub")
install(FILES osc2/controller.hpp osc2/driver.hpp
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/arm/osc2")
