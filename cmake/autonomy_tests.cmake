# Register unit tests against libautonomy.

function(autonomy_add_unit_tests)
  if(NOT BUILD_TEST)
    return()
  endif()

  set(TEST_LIB autonomy_test_library)
  add_library(${TEST_LIB} ${TEST_LIBRARY_HDRS} ${TEST_LIBRARY_SRCS})
  target_include_directories(${TEST_LIB} SYSTEM PRIVATE "${GMOCK_INCLUDE_DIRS}")
  target_link_libraries(${TEST_LIB} PUBLIC ${GMOCK_LIBRARY} ${PROJECT_NAME})

  foreach(ABS_FIL ${ALL_TESTS})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
    string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")

    google_test("${TEST_TARGET_NAME}" ${ABS_FIL})
    add_dependencies("${TEST_TARGET_NAME}" ${PROJECT_NAME})
    if(BUILD_GRPC)
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC grpc++ grpc)
    endif()
    if(BUILD_PROMETHEUS)
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC
        prometheus-cpp-core prometheus-cpp-pull)
    endif()
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${TEST_LIB})
  endforeach()
endfunction()
