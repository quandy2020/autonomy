autocmake_library(autodriver_l1w SHARED
  SOURCES
    driver.cpp
    l1w_component.cpp)
target_link_libraries(autodriver_l1w PUBLIC autodriver PRIVATE autolink)

file(GLOB _l1w_headers CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${_l1w_headers}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/chassis/l1w")

if(AUTODRIVER_WITH_GENISOM_L1W)
  autocmake_find(GenisomL1w PRIVATE)
  if(GenisomL1w_FOUND)
    autocmake_link(autodriver_l1w PRIVATE GenisomL1w)
    target_compile_definitions(autodriver_l1w PUBLIC AUTODRIVER_HAVE_GENISOM_L1W)
  endif()
endif()
