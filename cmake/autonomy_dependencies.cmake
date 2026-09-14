# Link helpers: minimal core + optional feature packs.
# Known FEATURES:
#   pcl slam bt grpc inference osqp cairo boost_iostreams ipopt foxglove
#   prometheus lua

include_guard(GLOBAL)

function(autonomy_link_core target)
  if(NOT DEFINED AUTONOMY_WORKSPACE_ROOT)
    set(AUTONOMY_WORKSPACE_ROOT "${PROJECT_SOURCE_DIR}")
  endif()
  # Minimal shared surface used by nearly every module.
  target_include_directories(${target} BEFORE PUBLIC
    $<BUILD_INTERFACE:${AUTONOMY_WORKSPACE_ROOT}/autolink>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${AUTONOMY_WORKSPACE_ROOT}>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/automsgs/proto/gen>
    $<INSTALL_INTERFACE:include>
  )
  target_include_directories(${target} SYSTEM PUBLIC
    ${Protobuf_INCLUDE_DIRS}
    "${EIGEN3_INCLUDE_DIR}"
    "${CERES_INCLUDE_DIRS}"
    "${OpenCV_INCLUDE_DIRS}"
  )
  target_link_libraries(${target} PUBLIC
    protobuf::libprotobuf
    automsgs
    ${EIGEN3_LIBRARIES}
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp::yaml-cpp
    glog
    gflags::gflags
    autolink
    Threads::Threads
    nlohmann_json::nlohmann_json
    TBB::tbb
  )
endfunction()

function(autonomy_link_feature target)
  foreach(_feat IN LISTS ARGN)
    string(TOLOWER "${_feat}" _feat)
    if(_feat STREQUAL "pcl")
      target_include_directories(${target} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
      target_link_libraries(${target} PUBLIC ${PCL_LIBRARIES})
      if(PCL_DEFINITIONS)
        target_compile_definitions(${target} PUBLIC ${PCL_DEFINITIONS})
      endif()
      if(OpenMP_CXX_FOUND)
        target_compile_definitions(${target} PUBLIC GRID_MAP_PCL_OPENMP_FOUND=1)
        target_link_libraries(${target} PUBLIC OpenMP::OpenMP_CXX)
      endif()
    elseif(_feat STREQUAL "slam")
      target_include_directories(${target} SYSTEM PUBLIC ${SQLite3_INCLUDE_DIRS})
      target_link_libraries(${target} PUBLIC
        FBow::fbow
        g2o::core g2o::stuff
        g2o::types_sba g2o::types_sim3
        g2o::solver_dense g2o::solver_eigen
        ${SQLite3_LIBRARIES})
    elseif(_feat STREQUAL "bt")
      if(TARGET BT::behaviortree_cpp)
        target_link_libraries(${target} PUBLIC BT::behaviortree_cpp)
      elseif(TARGET behaviortree_cpp::behaviortree_cpp)
        target_link_libraries(${target} PUBLIC behaviortree_cpp::behaviortree_cpp)
      else()
        message(FATAL_ERROR "behaviortree_cpp target not found")
      endif()
    elseif(_feat STREQUAL "grpc")
      if(BUILD_GRPC)
        target_link_libraries(${target} PUBLIC grpc++ grpc)
      endif()
    elseif(_feat STREQUAL "inference")
      if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
        target_compile_definitions(${target} PUBLIC AUTONOMY_HAS_ONNXRUNTIME)
        target_include_directories(${target} SYSTEM PUBLIC
          ${OnnxRuntime_INCLUDE_DIRS})
        target_link_libraries(${target} PUBLIC ${OnnxRuntime_LIBRARIES})
      endif()
      if(BUILD_TENSORRT AND TensorRT_FOUND)
        target_compile_definitions(${target} PUBLIC AUTONOMY_HAS_TENSORRT)
        if(TensorRT_NVONNXPARSER_LIBRARY)
          target_compile_definitions(${target} PUBLIC
            AUTONOMY_HAS_TENSORRT_ONNX_PARSER)
        endif()
        target_include_directories(${target} SYSTEM PUBLIC
          ${TensorRT_INCLUDE_DIRS})
        target_link_libraries(${target} PUBLIC ${TensorRT_LIBRARIES})
        if(TARGET CUDA::cudart)
          target_link_libraries(${target} PUBLIC CUDA::cudart)
        endif()
      endif()
    elseif(_feat STREQUAL "osqp")
      target_include_directories(${target} SYSTEM PUBLIC "${OSQP_INCLUDE_DIRS}")
      target_link_libraries(${target} PUBLIC ${OSQP_LIBRARIES})
    elseif(_feat STREQUAL "cairo")
      target_link_libraries(${target} PUBLIC PkgConfig::CAIRO)
    elseif(_feat STREQUAL "boost_iostreams")
      target_link_libraries(${target} PUBLIC Boost::iostreams)
    elseif(_feat STREQUAL "ipopt")
      if(Ipopt_FOUND)
        target_include_directories(${target} SYSTEM PUBLIC
          ${IPOPT_INCLUDE_DIRS})
        target_link_libraries(${target} PUBLIC ${IPOPT_LIBRARIES})
      endif()
    elseif(_feat STREQUAL "foxglove")
      if(foxglove-sdk_FOUND)
        target_link_libraries(${target} PUBLIC
          foxglove-sdk::foxglove_cpp_shared ZLIB::ZLIB)
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
          get_target_property(_viz_srcs ${target} SOURCES)
          if(_viz_srcs)
            set_source_files_properties(${_viz_srcs} PROPERTIES
              COMPILE_OPTIONS "-Wno-error=maybe-uninitialized")
          endif()
        endif()
      endif()
    elseif(_feat STREQUAL "prometheus")
      if(BUILD_PROMETHEUS)
        target_compile_definitions(${target} PRIVATE USE_PROMETHEUS=1)
        target_link_libraries(${target} PRIVATE
          prometheus-cpp-core prometheus-cpp-pull)
      endif()
    elseif(_feat STREQUAL "lua")
      if(NOT LUA_INCLUDE_DIR OR NOT LUA_LIBRARIES)
        message(FATAL_ERROR "FEATURES lua requires LuaGoogle (find_package)")
      endif()
      target_include_directories(${target} SYSTEM PUBLIC "${LUA_INCLUDE_DIR}")
      target_link_libraries(${target} PUBLIC ${LUA_LIBRARIES})
    else()
      message(FATAL_ERROR "autonomy_link_feature: unknown '${_feat}'")
    endif()
  endforeach()
endfunction()
