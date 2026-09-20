# @file autonomy_deps.cmake
# @brief Third-party package discovery (find_package groups) and link helpers.
#
# @par Main APIs
#   - autonomy_collect_required_package_groups()
#   - autonomy_find_dependencies() (macro; find results stay visible to link_*)
#   - autonomy_link_core() / autonomy_link_feature()
#
# @par FEATURES keywords (autonomy_link_feature)
#   pcl, slam, bt, grpc, grpc_reflection, otel, inference, osqp, cairo,
#   boost_iostreams, ipopt, foxglove, prometheus, lua, kdl, fcl, ompl, ruckig,
#   urdfdom, pinocchio, vhacd, trac_ik

include_guard(GLOBAL)

# cmake -P / older hosts may not have CMP0057 NEW by default.
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

if(NOT DEFINED AUTONOMY_BUILD_COMMON_OSQP)
  option(AUTONOMY_BUILD_COMMON_OSQP
    "Build the OSQP-backed common MPC library" ON)
endif()

# @brief Derive required package-group names from @c AUTONOMY_ENABLED_MODULES
#        and product options.
# @param _out_variable Output variable name; value is a list of groups
#        (core, map, grpc, ...).
function(autonomy_collect_required_package_groups _out_variable)
  set(_groups "")
  if("common" IN_LIST AUTONOMY_ENABLED_MODULES)
    list(APPEND _groups core common_math)
    if(AUTONOMY_BUILD_COMMON_OSQP)
      list(APPEND _groups osqp)
    endif()
  endif()
  if("map" IN_LIST AUTONOMY_ENABLED_MODULES
      OR "localization" IN_LIST AUTONOMY_ENABLED_MODULES
      OR "perception" IN_LIST AUTONOMY_ENABLED_MODULES
      OR ((BUILD_ONNXRUNTIME OR BUILD_TENSORRT)
          AND "common" IN_LIST AUTONOMY_ENABLED_MODULES))
    list(APPEND _groups common_vision)
  endif()
  if("map" IN_LIST AUTONOMY_ENABLED_MODULES)
    list(APPEND _groups map)
  endif()
  if("control" IN_LIST AUTONOMY_ENABLED_MODULES)
    list(APPEND _groups control)
  endif()
  if("task" IN_LIST AUTONOMY_ENABLED_MODULES)
    list(APPEND _groups task)
  endif()
  if("localization" IN_LIST AUTONOMY_ENABLED_MODULES)
    list(APPEND _groups localization)
  endif()
  if(BUILD_GRPC AND "bridge" IN_LIST AUTONOMY_ENABLED_MODULES)
    list(APPEND _groups grpc)
  endif()
  if(BUILD_AUTOVIZ)
    list(APPEND _groups autoviz)
  endif()
  if(BUILD_PROMETHEUS AND "system" IN_LIST AUTONOMY_ENABLED_MODULES)
    list(APPEND _groups prometheus)
  endif()
  if(BUILD_SHERPA_ONNX AND "audio" IN_LIST AUTONOMY_ENABLED_MODULES)
    list(APPEND _groups sherpa_onnx)
  endif()
  if((BUILD_ONNXRUNTIME OR BUILD_TENSORRT)
      AND ("common" IN_LIST AUTONOMY_ENABLED_MODULES
           OR "perception" IN_LIST AUTONOMY_ENABLED_MODULES))
    list(APPEND _groups inference)
  endif()
  if("visualization" IN_LIST AUTONOMY_ENABLED_MODULES)
    list(APPEND _groups foxglove)
  endif()
  if("manipulation" IN_LIST AUTONOMY_ENABLED_MODULES)
    list(APPEND _groups manipulation)
  endif()
  list(REMOVE_DUPLICATES _groups)
  set(${_out_variable} "${_groups}" PARENT_SCOPE)
endfunction()

# @brief Batch find_package by package group (includes ordering such as Qt6
#        before PCL).
# @note Implemented as a macro so Protobuf, PCL, etc. remain visible to callers.
macro(autonomy_find_dependencies)
  autonomy_collect_required_package_groups(_required_groups)

  # PCL → VTK::GUISupportQt pulls Qt5 and claims versionless Qt::Core. Find Qt6
  # first so Qt::* aliases belong to Qt6 (otherwise Qt6CoreVersionlessTargets fails).
  # Headless / aarch64 boards often lack Qt6: soft-disable autoviz instead of hard fail.
  if("autoviz" IN_LIST _required_groups)
    find_package(Qt6 QUIET COMPONENTS
      Core Gui Widgets OpenGLWidgets OpenGL Xml Svg Network)
    if(NOT Qt6_FOUND)
      message(WARNING
        "Qt6 not found; disabling BUILD_AUTOVIZ "
        "(install Qt6 or pass -DBUILD_AUTOVIZ=OFF).")
      set(BUILD_AUTOVIZ OFF CACHE BOOL
        "Build native 3D visualization tool (autoviz)" FORCE)
    endif()
    list(REMOVE_ITEM _required_groups autoviz)
  endif()

  foreach(_group IN LISTS _required_groups)
    if(_group STREQUAL "core")
      find_package(Eigen3 REQUIRED CONFIG)
      find_package(nlohmann_json REQUIRED)
      set(GFLAGS_USE_TARGET_NAMESPACE TRUE)
      find_package(gflags CONFIG REQUIRED)
      # CeresConfig expects a bare `gflags` target (not only gflags::gflags).
      if(TARGET gflags::gflags AND NOT TARGET gflags)
        add_library(gflags INTERFACE IMPORTED)
        set_property(TARGET gflags PROPERTY
          INTERFACE_LINK_LIBRARIES gflags::gflags)
      endif()
      # Upstream exports package name `glog` (not `Glog`).
      find_package(glog CONFIG REQUIRED)
      if(TARGET glog::glog AND NOT TARGET glog)
        add_library(glog ALIAS glog::glog)
      endif()
      include(EnsureProtobuf319)
      find_package(yaml-cpp REQUIRED)
      # Ubuntu/debian export the target as `yaml-cpp`; newer packages use
      # `yaml-cpp::yaml-cpp`. Normalize so link helpers can use one name.
      if(TARGET yaml-cpp AND NOT TARGET yaml-cpp::yaml-cpp)
        add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
      endif()
      find_package(Threads REQUIRED)
    elseif(_group STREQUAL "common_math")
      find_package(Ceres REQUIRED CONFIG)
    elseif(_group STREQUAL "common_vision")
      find_package(OpenCV REQUIRED)
      if(NOT TARGET autonomy_opencv)
        add_library(autonomy_opencv INTERFACE)
        target_include_directories(autonomy_opencv SYSTEM INTERFACE
          $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIRS}>)
        target_link_libraries(autonomy_opencv INTERFACE ${OpenCV_LIBS})
        add_library(autonomy::opencv ALIAS autonomy_opencv)
        set_property(GLOBAL APPEND
          PROPERTY AUTONOMY_MODULE_TARGETS autonomy_opencv)
      endif()
    elseif(_group STREQUAL "osqp")
      find_package(OSQP REQUIRED)
    elseif(_group STREQUAL "map")
      find_package(PCL REQUIRED COMPONENTS
        common features filters io kdtree segmentation surface)
      find_package(TBB REQUIRED)
      find_package(OpenMP QUIET)
    elseif(_group STREQUAL "control")
      find_package(Ipopt QUIET)
    elseif(_group STREQUAL "manipulation")
      find_package(OrocosKDL QUIET)
      if(OrocosKDL_FOUND)
        message(STATUS "OrocosKDL found (manipulation FK/IK)")
      else()
        message(STATUS "OrocosKDL not found; manipulation uses StubKinematics")
      endif()
      find_package(FCL QUIET)
      find_package(OMPL QUIET)
      find_package(Ruckig QUIET)
      find_package(urdfdom QUIET)
      find_package(Pinocchio QUIET)
      find_package(VHACD QUIET)
      find_package(TRAC_IK QUIET)
      find_package(Octomap QUIET)
      if(FCL_FOUND)
        message(STATUS "FCL found (manipulation collision)")
      endif()
      if(OMPL_FOUND)
        message(STATUS "OMPL found (manipulation planner adapter)")
      endif()
      if(Ruckig_FOUND)
        message(STATUS "Ruckig found (trajectory smoother)")
      endif()
      if(urdfdom_FOUND)
        message(STATUS "urdfdom found (optional URDF parser)")
      endif()
      if(Pinocchio_FOUND)
        message(STATUS "Pinocchio found (manipulation dynamics)")
      endif()
      if(VHACD_FOUND)
        message(STATUS "VHACD found (online multi-convex)")
      endif()
      if(TRAC_IK_FOUND)
        message(STATUS "TRAC_IK found (manipulation IK)")
      endif()
      if(Octomap_FOUND)
        message(STATUS "Octomap found (OcTree occupancy decode)")
      endif()
    elseif(_group STREQUAL "task")
      # Prefer a plain CMake install (/usr/local from install_behaviortree_cpp.sh)
      # over ROS ament packages that pull broken ament_package on some boards.
      find_package(behaviortree_cpp QUIET
        PATHS /usr/local
        NO_DEFAULT_PATH)
      if(NOT behaviortree_cpp_FOUND)
        find_package(behaviortree_cpp QUIET)
      endif()
      if(NOT behaviortree_cpp_FOUND)
        message(WARNING
          "behaviortree_cpp not found; disabling task/system/bridge modules.")
        set(AUTONOMY_BUILD_TASK OFF CACHE BOOL
          "Build the autonomy task module" FORCE)
        set(AUTONOMY_BUILD_SYSTEM OFF CACHE BOOL
          "Build the autonomy system module" FORCE)
        set(AUTONOMY_BUILD_BRIDGE OFF CACHE BOOL
          "Build the autonomy bridge module" FORCE)
        set(BUILD_GRPC OFF CACHE BOOL "Build autonomy gRPC support" FORCE)
        list(REMOVE_ITEM AUTONOMY_ENABLED_MODULES task system bridge)
      endif()
    elseif(_group STREQUAL "localization")
      find_package(LuaGoogle QUIET)
      find_package(FBow QUIET)
      find_package(G2o QUIET)
      if(NOT LuaGoogle_FOUND OR NOT FBow_FOUND OR NOT G2o_FOUND)
        message(WARNING
          "localization deps missing (LuaGoogle/FBow/G2O); "
          "disabling AUTONOMY_BUILD_LOCALIZATION.")
        set(AUTONOMY_BUILD_LOCALIZATION OFF CACHE BOOL
          "Build the autonomy localization module" FORCE)
        list(REMOVE_ITEM AUTONOMY_ENABLED_MODULES localization)
      else()
        find_package(SQLite3 REQUIRED)
        find_package(Boost REQUIRED COMPONENTS iostreams)
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(CAIRO REQUIRED IMPORTED_TARGET cairo)
      endif()
    elseif(_group STREQUAL "grpc")
      if(BUILD_GRPC AND "bridge" IN_LIST AUTONOMY_ENABLED_MODULES)
        # Prefer CMake config (source / install_grpc.sh); fall back to
        # cmake/modules/FindgRPC.cmake for Ubuntu apt libgrpc++-dev.
        find_package(gRPC CONFIG QUIET)
        if(NOT gRPC_FOUND)
          find_package(gRPC MODULE QUIET)
        endif()
        if(NOT gRPC_FOUND)
          message(WARNING
            "gRPC not found (no gRPCConfig.cmake / FindgRPC); "
            "disabling BUILD_GRPC and bridge. "
            "Install libgrpc++-dev or run docker/install/install_grpc.sh.")
          set(BUILD_GRPC OFF CACHE BOOL "Build autonomy gRPC support" FORCE)
          set(AUTONOMY_BUILD_BRIDGE OFF CACHE BOOL
            "Build the autonomy bridge module" FORCE)
          list(REMOVE_ITEM AUTONOMY_ENABLED_MODULES bridge)
        endif()
      else()
        message(STATUS "Skipping gRPC (bridge/BUILD_GRPC disabled)")
      endif()
    elseif(_group STREQUAL "prometheus")
      find_package(prometheus-cpp CONFIG REQUIRED)
    elseif(_group STREQUAL "sherpa_onnx")
      find_package(SherpaOnnx QUIET)
      if(SherpaOnnx_FOUND)
        message(STATUS "SherpaOnnx found: ${SherpaOnnx_INCLUDE_DIRS}")
      else()
        message(STATUS "SherpaOnnx not found; audio ASR uses stub engine")
      endif()
    elseif(_group STREQUAL "inference")
      if(BUILD_ONNXRUNTIME)
        find_package(OnnxRuntime QUIET)
      endif()
      if(BUILD_TENSORRT)
        find_package(TensorRT QUIET)
        find_package(CUDAToolkit QUIET)
        if(TensorRT_FOUND)
          message(STATUS "TensorRT found: ${TensorRT_INCLUDE_DIRS}")
        else()
          message(STATUS "TensorRT not found; tensorrt backend stubbed")
        endif()
      endif()
    elseif(_group STREQUAL "foxglove")
      find_package(foxglove-sdk CONFIG QUIET)
      if(foxglove-sdk_FOUND)
        find_package(ZLIB REQUIRED)
      endif()
    else()
      message(FATAL_ERROR
        "autonomy_find_dependencies: unknown package group '${_group}'")
    endif()
  endforeach()
endmacro()

# @brief Link the shared "core" dependency surface onto a domain library target
#        (Eigen, Protobuf, autolink, yaml-cpp, ...).
# @param target Existing CMake target name.
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
  # Skip empty entries: CMake treats "" as CMAKE_CURRENT_SOURCE_DIR, which then
  # breaks install(EXPORT) ("prefixed in the source directory").
  set(_core_sys_includes "")
  foreach(_inc IN LISTS
      Protobuf_INCLUDE_DIRS EIGEN3_INCLUDE_DIR CERES_INCLUDE_DIRS
      OpenCV_INCLUDE_DIRS)
    if(_inc)
      list(APPEND _core_sys_includes "${_inc}")
    endif()
  endforeach()
  if(_core_sys_includes)
    list(REMOVE_DUPLICATES _core_sys_includes)
    target_include_directories(${target} SYSTEM PUBLIC ${_core_sys_includes})
  endif()
  unset(_core_sys_includes)

  set(_core_libs
    protobuf::libprotobuf
    automsgs
    yaml-cpp::yaml-cpp
    gflags::gflags
    autolink
    Threads::Threads
    nlohmann_json::nlohmann_json
    TBB::tbb)
  if(TARGET Eigen3::Eigen)
    list(APPEND _core_libs Eigen3::Eigen)
  elseif(EIGEN3_LIBRARIES)
    list(APPEND _core_libs ${EIGEN3_LIBRARIES})
  endif()
  if(TARGET Ceres::ceres)
    list(APPEND _core_libs Ceres::ceres)
  elseif(CERES_LIBRARIES)
    list(APPEND _core_libs ${CERES_LIBRARIES})
  endif()
  if(TARGET autonomy_opencv)
    list(APPEND _core_libs autonomy_opencv)
  elseif(OpenCV_LIBS)
    list(APPEND _core_libs ${OpenCV_LIBS})
  endif()
  if(TARGET glog::glog)
    list(APPEND _core_libs glog::glog)
  elseif(TARGET glog)
    list(APPEND _core_libs glog)
  endif()
  target_link_libraries(${target} PUBLIC ${_core_libs})
  unset(_core_libs)
endfunction()

# @brief Link optional dependencies from a FEATURES list (PCL, SLAM, gRPC,
#        inference backends, ...).
# @param target Target name.
# @param ... FEATURES strings (see file-level @par FEATURES).
function(autonomy_link_feature target)
  foreach(_feat IN LISTS ARGN)
    string(TOLOWER "${_feat}" _feat)
    if(_feat STREQUAL "pcl")
      if(PCL_INCLUDE_DIRS)
        target_include_directories(${target} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
      endif()
      target_link_libraries(${target} PUBLIC ${PCL_LIBRARIES})
      if(PCL_DEFINITIONS)
        target_compile_definitions(${target} PUBLIC ${PCL_DEFINITIONS})
      endif()
      if(OpenMP_CXX_FOUND)
        target_compile_definitions(${target} PUBLIC GRID_MAP_PCL_OPENMP_FOUND=1)
        target_link_libraries(${target} PUBLIC OpenMP::OpenMP_CXX)
      endif()
    elseif(_feat STREQUAL "slam")
      if(SQLite3_INCLUDE_DIRS)
        target_include_directories(${target} SYSTEM PUBLIC ${SQLite3_INCLUDE_DIRS})
      endif()
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
    elseif(_feat STREQUAL "grpc_reflection")
      # Optional: enables InitProtoReflectionServerBuilderPlugin (async_grpc).
      if(BUILD_GRPC)
        if(TARGET gRPC::grpc++_reflection)
          target_link_libraries(${target} PUBLIC gRPC::grpc++_reflection)
          target_compile_definitions(${target} PUBLIC
            AUTONOMY_HAVE_GRPC_REFLECTION=1)
          message(STATUS
            "FEATURES grpc_reflection: linked gRPC::grpc++_reflection")
        elseif(GRPC_GRPC++_REFLECTION_LIBRARY)
          target_link_libraries(${target} PUBLIC
            ${GRPC_GRPC++_REFLECTION_LIBRARY})
          target_compile_definitions(${target} PUBLIC
            AUTONOMY_HAVE_GRPC_REFLECTION=1)
          message(STATUS
            "FEATURES grpc_reflection: linked ${GRPC_GRPC++_REFLECTION_LIBRARY}")
        else()
          message(STATUS
            "FEATURES grpc_reflection: library not found; reflection disabled")
        endif()
      endif()
    elseif(_feat STREQUAL "otel")
      # Optional OpenTelemetry C++ SDK. Phase-1 Bridge uses NoopTracerProvider
      # when the SDK is absent; define AUTONOMY_HAVE_OTEL only when present.
      find_package(opentelemetry-cpp CONFIG QUIET)
      if(opentelemetry-cpp_FOUND)
        target_compile_definitions(${target} PUBLIC AUTONOMY_HAVE_OTEL=1)
        if(TARGET opentelemetry-cpp::opentelemetry_trace)
          target_link_libraries(${target} PUBLIC
            opentelemetry-cpp::opentelemetry_trace)
        endif()
        message(STATUS "FEATURES otel: OpenTelemetry C++ SDK found")
      else()
        message(STATUS
          "FEATURES otel: OpenTelemetry C++ SDK not found; using no-op tracer")
      endif()
    elseif(_feat STREQUAL "inference")
      if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
        target_compile_definitions(${target} PUBLIC AUTONOMY_HAS_ONNXRUNTIME)
        if(OnnxRuntime_INCLUDE_DIRS)
          target_include_directories(${target} SYSTEM PUBLIC
            ${OnnxRuntime_INCLUDE_DIRS})
        endif()
        target_link_libraries(${target} PUBLIC ${OnnxRuntime_LIBRARIES})
      endif()
      if(BUILD_TENSORRT AND TensorRT_FOUND)
        target_compile_definitions(${target} PUBLIC AUTONOMY_HAS_TENSORRT)
        if(TensorRT_NVONNXPARSER_LIBRARY)
          target_compile_definitions(${target} PUBLIC
            AUTONOMY_HAS_TENSORRT_ONNX_PARSER)
        endif()
        if(TensorRT_INCLUDE_DIRS)
          target_include_directories(${target} SYSTEM PUBLIC
            ${TensorRT_INCLUDE_DIRS})
        endif()
        target_link_libraries(${target} PUBLIC ${TensorRT_LIBRARIES})
        if(TARGET CUDA::cudart)
          target_link_libraries(${target} PUBLIC CUDA::cudart)
        endif()
      endif()
    elseif(_feat STREQUAL "osqp")
      if(TARGET OSQP::OSQP)
        target_link_libraries(${target} PUBLIC OSQP::OSQP)
      else()
        if(OSQP_INCLUDE_DIRS)
          target_include_directories(${target} SYSTEM PUBLIC
            "${OSQP_INCLUDE_DIRS}")
        endif()
        target_link_libraries(${target} PUBLIC ${OSQP_LIBRARIES})
      endif()
    elseif(_feat STREQUAL "cairo")
      target_link_libraries(${target} PUBLIC PkgConfig::CAIRO)
    elseif(_feat STREQUAL "boost_iostreams")
      target_link_libraries(${target} PUBLIC Boost::iostreams)
    elseif(_feat STREQUAL "ipopt")
      if(Ipopt_FOUND)
        if(IPOPT_INCLUDE_DIRS)
          target_include_directories(${target} SYSTEM PUBLIC
            ${IPOPT_INCLUDE_DIRS})
        endif()
        target_link_libraries(${target} PUBLIC ${IPOPT_LIBRARIES})
      endif()
    elseif(_feat STREQUAL "kdl")
      if(OrocosKDL_FOUND)
        target_compile_definitions(${target} PUBLIC AUTONOMY_HAS_KDL)
        if(TARGET OrocosKDL::orocos-kdl)
          target_link_libraries(${target} PUBLIC OrocosKDL::orocos-kdl)
        else()
          if(OrocosKDL_INCLUDE_DIRS)
            target_include_directories(${target} SYSTEM PUBLIC
              ${OrocosKDL_INCLUDE_DIRS})
          endif()
          target_link_libraries(${target} PUBLIC ${OrocosKDL_LIBRARIES})
        endif()
      endif()
    elseif(_feat STREQUAL "fcl")
      if(FCL_FOUND)
        target_compile_definitions(${target} PUBLIC AUTONOMY_HAS_FCL)
        if(TARGET FCL::fcl)
          target_link_libraries(${target} PUBLIC FCL::fcl)
        elseif(TARGET fcl)
          target_link_libraries(${target} PUBLIC fcl)
        else()
          if(FCL_INCLUDE_DIRS)
            target_include_directories(${target} SYSTEM PUBLIC
              ${FCL_INCLUDE_DIRS})
          endif()
          target_link_libraries(${target} PUBLIC ${FCL_LIBRARIES})
        endif()
      endif()
    elseif(_feat STREQUAL "ompl")
      if(OMPL_FOUND)
        target_compile_definitions(${target} PUBLIC AUTONOMY_HAS_OMPL)
        if(TARGET OMPL::OMPL)
          target_link_libraries(${target} PUBLIC OMPL::OMPL)
        elseif(TARGET ompl::ompl)
          target_link_libraries(${target} PUBLIC ompl::ompl)
        else()
          if(OMPL_INCLUDE_DIRS)
            target_include_directories(${target} SYSTEM PUBLIC
              ${OMPL_INCLUDE_DIRS})
          endif()
          target_link_libraries(${target} PUBLIC ${OMPL_LIBRARIES})
        endif()
      endif()
    elseif(_feat STREQUAL "ruckig")
      if(Ruckig_FOUND)
        target_compile_definitions(${target} PUBLIC AUTONOMY_HAS_RUCKIG)
        if(TARGET ruckig::ruckig)
          target_link_libraries(${target} PUBLIC ruckig::ruckig)
        else()
          if(RUCKIG_INCLUDE_DIRS)
            target_include_directories(${target} SYSTEM PUBLIC
              ${RUCKIG_INCLUDE_DIRS})
          endif()
          target_link_libraries(${target} PUBLIC ${RUCKIG_LIBRARIES})
        endif()
      endif()
    elseif(_feat STREQUAL "urdfdom")
      if(urdfdom_FOUND)
        target_compile_definitions(${target} PUBLIC AUTONOMY_HAS_URDFDOM)
        # Debian/Ubuntu urdfdom exports urdfdom::urdfdom_model (not urdf_parser).
        if(TARGET urdfdom::urdfdom_model)
          target_link_libraries(${target} PUBLIC urdfdom::urdfdom_model)
        elseif(urdfdom_LIBRARIES)
          target_include_directories(${target} SYSTEM PUBLIC ${urdfdom_INCLUDE_DIRS})
          target_link_libraries(${target} PUBLIC ${urdfdom_LIBRARIES})
        endif()
      endif()
    elseif(_feat STREQUAL "pinocchio")
      if(Pinocchio_FOUND)
        target_compile_definitions(${target} PUBLIC AUTONOMY_HAS_PINOCCHIO)
        # pinocchioTargets sets BOOST_MPL_LIMIT_{VECTOR,LIST}_SIZE=30 but not
        # BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS. With the default preprocessed
        # mpl::vector (arity 20), OMPL → Boost.MultiIndex then fails:
        # "wrong number of template arguments (30, should be at most 20)".
        target_compile_definitions(${target} PUBLIC
          BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
        if(TARGET pinocchio::pinocchio)
          target_link_libraries(${target} PUBLIC pinocchio::pinocchio)
        else()
          if(PINOCCHIO_INCLUDE_DIRS)
            target_include_directories(${target} SYSTEM PUBLIC
              ${PINOCCHIO_INCLUDE_DIRS})
          endif()
          target_link_libraries(${target} PUBLIC ${PINOCCHIO_LIBRARIES})
        endif()
      endif()
    elseif(_feat STREQUAL "vhacd")
      if(VHACD_FOUND)
        target_compile_definitions(${target} PUBLIC AUTONOMY_HAS_VHACD)
        if(TARGET VHACD::VHACD)
          target_link_libraries(${target} PUBLIC VHACD::VHACD)
        else()
          if(VHACD_INCLUDE_DIRS)
            target_include_directories(${target} SYSTEM PUBLIC
              ${VHACD_INCLUDE_DIRS})
          endif()
          target_link_libraries(${target} PUBLIC ${VHACD_LIBRARIES})
        endif()
      endif()
    elseif(_feat STREQUAL "trac_ik")
      if(TRAC_IK_FOUND)
        target_compile_definitions(${target} PUBLIC AUTONOMY_HAS_TRAC_IK)
        if(TARGET TRAC_IK::TRAC_IK)
          target_link_libraries(${target} PUBLIC TRAC_IK::TRAC_IK)
        else()
          if(TRAC_IK_INCLUDE_DIRS)
            target_include_directories(${target} SYSTEM PUBLIC
              ${TRAC_IK_INCLUDE_DIRS})
          endif()
          target_link_libraries(${target} PUBLIC ${TRAC_IK_LIBRARIES})
        endif()
      endif()
    elseif(_feat STREQUAL "octomap")
      if(Octomap_FOUND)
        target_compile_definitions(${target} PUBLIC AUTONOMY_HAS_OCTOMAP)
        if(TARGET octomap::octomap)
          target_link_libraries(${target} PUBLIC octomap::octomap)
        else()
          if(OCTOMAP_INCLUDE_DIRS)
            target_include_directories(${target} SYSTEM PUBLIC
              ${OCTOMAP_INCLUDE_DIRS})
          endif()
          target_link_libraries(${target} PUBLIC ${OCTOMAP_LIBRARIES})
        endif()
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
