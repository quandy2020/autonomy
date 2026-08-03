# BICMap example catalog + headless test runner (rviz-style display tests).

find_package(Python3 COMPONENTS Interpreter QUIET)

if(Python3_FOUND)
  add_test(
    NAME autoviz_examples_catalog_verify
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/verify_catalog.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

if(TARGET autoviz_bicmap_examples_test)
  if(DEFINED ENV{DISPLAY})
    set(_autoviz_examples_display "$ENV{DISPLAY}")
  else()
    set(_autoviz_examples_display ":1")
  endif()
  add_custom_target(autoviz_examples_tests
    COMMAND ${CMAKE_COMMAND} -E env
      QT_QPA_PLATFORM=offscreen
      DISPLAY=${_autoviz_examples_display}
      $<TARGET_FILE:autoviz_bicmap_examples_test>
    DEPENDS autoviz_bicmap_examples_test
    COMMENT "Run all BICMap example headless display tests")
endif()

install(FILES catalog.yaml README.md
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/autonomy/autoviz/examples)
install(PROGRAMS verify_catalog.py run_all_tests.sh
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/autonomy/autoviz/examples)
