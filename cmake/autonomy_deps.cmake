# Link dependencies for libautonomy (${PROJECT_NAME}).

function(autonomy_link_dependencies target)
  target_include_directories(${target} BEFORE PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/autolink>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/automsgs/proto/gen>
    $<INSTALL_INTERFACE:include>
  )

  # --- Required ---
  target_include_directories(${target} SYSTEM PUBLIC
    ${Protobuf_INCLUDE_DIRS}
    "${EIGEN3_INCLUDE_DIR}"
    "${CERES_INCLUDE_DIRS}"
    "${LUA_INCLUDE_DIR}"
    "${OpenCV_INCLUDE_DIRS}"
    "${OSQP_INCLUDE_DIRS}"
    ${SQLite3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(${target} PUBLIC
    protobuf::libprotobuf
    automsgs
    ${EIGEN3_LIBRARIES}
    ${CERES_LIBRARIES}
    ${LUA_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp
    ${OSQP_LIBRARIES}
    glog
    gflags
    FBow::fbow
    g2o::core
    g2o::stuff
    g2o::types_sba
    g2o::types_sim3
    g2o::solver_dense
    g2o::solver_eigen
    ${SQLite3_LIBRARIES}
    autolink
    Boost::iostreams
    PkgConfig::CAIRO
    Threads::Threads
    nlohmann_json::nlohmann_json
    ${PCL_LIBRARIES}
    TBB::tbb
  )
  if(PCL_DEFINITIONS)
    target_compile_definitions(${target} PUBLIC ${PCL_DEFINITIONS})
  endif()
  if(OpenMP_CXX_FOUND)
    target_compile_definitions(${target} PUBLIC GRID_MAP_PCL_OPENMP_FOUND=1)
    target_link_libraries(${target} PUBLIC OpenMP::OpenMP_CXX)
  endif()

  if(TARGET BT::behaviortree_cpp)
    target_link_libraries(${target} PUBLIC BT::behaviortree_cpp)
  elseif(TARGET behaviortree_cpp::behaviortree_cpp)
    target_link_libraries(${target} PUBLIC behaviortree_cpp::behaviortree_cpp)
  else()
    message(FATAL_ERROR
      "behaviortree_cpp imported target not found after find_package")
  endif()

  # --- Optional ---
  if(BUILD_GRPC)
    target_link_libraries(${target} PUBLIC grpc++ grpc)
  endif()

  if(BUILD_PROMETHEUS)
    target_compile_definitions(${target} PRIVATE USE_PROMETHEUS=1)
    target_link_libraries(${target} PRIVATE
      prometheus-cpp-core prometheus-cpp-pull)
  endif()

  if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
    target_include_directories(${target} SYSTEM PUBLIC
      ${OnnxRuntime_INCLUDE_DIRS})
    target_link_libraries(${target} PUBLIC ${OnnxRuntime_LIBRARIES})
  endif()

  if(Ipopt_FOUND)
    target_include_directories(${target} SYSTEM PUBLIC
      ${IPOPT_INCLUDE_DIRS})
    target_link_libraries(${target} PUBLIC ${IPOPT_LIBRARIES})
  endif()

  if(foxglove-sdk_FOUND)
    target_link_libraries(${target} PUBLIC
      foxglove-sdk::foxglove_cpp_shared
      ZLIB::ZLIB)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set_source_files_properties(${VISUALIZATION_SRCS} PROPERTIES
        COMPILE_OPTIONS "-Wno-error=maybe-uninitialized")
    endif()
  else()
    message(WARNING
      "foxglove-sdk not found; visualization sources excluded from libautonomy. "
      "Set CMAKE_PREFIX_PATH or install foxglove-sdk to enable.")
  endif()
endfunction()
