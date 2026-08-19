# Install rules for standalone autodriver builds.

install(
  TARGETS autodriver
          autodriver_imu
          autodriver_gps
          autodriver_camera
          autodriver_lidar
          autodriver_range
  EXPORT autodriverTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  DIRECTORY ${AUTODRIVER_ROOT_DIR}/autodriver/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/autodriver
  FILES_MATCHING PATTERN "*.hpp"
)

if(AUTODRIVER_BUILD_EXAMPLES)
  install(TARGETS autodriver_demo RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

install(
  EXPORT autodriverTargets
  FILE autodriverTargets.cmake
  NAMESPACE autodriver::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/autodriver
)
