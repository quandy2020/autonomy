autocmake_library(autodriver_jetauto SHARED
  SOURCES
    rrc_protocol.cpp
    kinematics.cpp
    driver.cpp
    jetauto_component.cpp)
target_link_libraries(autodriver_jetauto PUBLIC autodriver PRIVATE autolink)

file(GLOB _jetauto_headers CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${_jetauto_headers}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/chassis/jetauto")
