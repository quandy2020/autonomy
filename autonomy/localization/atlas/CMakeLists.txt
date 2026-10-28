cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(autonomy_localization_atlas LANGUAGES CXX)
endif()

option(ATLAS_BUILD_EXAMPLES "Build Atlas dataset examples" OFF)
option(ATLAS_BUILD_TESTS "Build Atlas unit tests" OFF)
option(ATLAS_ENABLE_OPENMP "Use bounded OpenMP parallelism in Atlas" ON)

if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 REQUIRED)
endif()
if(NOT TARGET Threads::Threads)
  find_package(Threads REQUIRED)
endif()
if(NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED COMPONENTS core features2d imgproc imgcodecs calib3d)
endif()
if(NOT TARGET OpenSSL::Crypto)
  find_package(OpenSSL REQUIRED COMPONENTS Crypto)
endif()

if(ATLAS_ENABLE_OPENMP)
  find_package(OpenMP QUIET COMPONENTS CXX)
  if(NOT OpenMP_CXX_FOUND)
    message(WARNING "Atlas OpenMP support requested but unavailable; using thread-pool fallback")
  endif()
endif()

if(NOT TARGET FBow::fbow AND NOT TARGET fbow::fbow AND NOT TARGET fbow)
  find_package(fbow CONFIG QUIET)
endif()
if(NOT TARGET FBow::fbow AND NOT TARGET fbow::fbow AND NOT TARGET fbow)
  find_package(FBow REQUIRED)
endif()
if(TARGET FBow::fbow)
  set(ATLAS_FBOW_TARGET FBow::fbow)
elseif(TARGET fbow::fbow)
  set(ATLAS_FBOW_TARGET fbow::fbow)
elseif(TARGET fbow)
  set(ATLAS_FBOW_TARGET fbow)
else()
  message(FATAL_ERROR "Atlas requires FBow")
endif()

if(NOT TARGET automsgs::automsgs AND NOT TARGET automsgs::core AND
   NOT TARGET automsgs)
  find_package(automsgs CONFIG REQUIRED)
endif()
if(TARGET automsgs::automsgs)
  set(ATLAS_AUTOMSGS_TARGET automsgs::automsgs)
elseif(TARGET automsgs::core)
  set(ATLAS_AUTOMSGS_TARGET automsgs::core)
elseif(TARGET automsgs)
  set(ATLAS_AUTOMSGS_TARGET automsgs)
else()
  message(FATAL_ERROR "Atlas requires AutoMsgs")
endif()

if(NOT TARGET Autolink::Autolink AND NOT TARGET autolink::autolink AND
   NOT TARGET autolink)
  find_package(Autolink CONFIG REQUIRED)
endif()
if(TARGET Autolink::Autolink)
  set(ATLAS_AUTOLINK_TARGET Autolink::Autolink)
elseif(TARGET autolink::autolink)
  set(ATLAS_AUTOLINK_TARGET autolink::autolink)
elseif(TARGET autolink)
  set(ATLAS_AUTOLINK_TARGET autolink)
else()
  message(FATAL_ERROR "Atlas requires Autolink")
endif()

if(TARGET autonomy_opencv)
  set(ATLAS_OPENCV_TARGET autonomy_opencv)
else()
  set(ATLAS_OPENCV_TARGET ${OpenCV_LIBS})
endif()

set(ATLAS_ROOT "${CMAKE_CURRENT_LIST_DIR}")
set(ATLAS_THIRDPARTY "${ATLAS_ROOT}/thirdparty")

add_library(atlas_thirdparty_g2o STATIC
  thirdparty/g2o/g2o/types/types_sba.cpp
  thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp
  thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp
  thirdparty/g2o/g2o/core/hyper_graph_action.cpp
  thirdparty/g2o/g2o/core/hyper_graph.cpp
  thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp
  thirdparty/g2o/g2o/core/matrix_structure.cpp
  thirdparty/g2o/g2o/core/batch_stats.cpp
  thirdparty/g2o/g2o/core/parameter.cpp
  thirdparty/g2o/g2o/core/cache.cpp
  thirdparty/g2o/g2o/core/optimizable_graph.cpp
  thirdparty/g2o/g2o/core/solver.cpp
  thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp
  thirdparty/g2o/g2o/core/estimate_propagator.cpp
  thirdparty/g2o/g2o/core/factory.cpp
  thirdparty/g2o/g2o/core/sparse_optimizer.cpp
  thirdparty/g2o/g2o/core/hyper_dijkstra.cpp
  thirdparty/g2o/g2o/core/parameter_container.cpp
  thirdparty/g2o/g2o/core/optimization_algorithm.cpp
  thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp
  thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp
  thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.cpp
  thirdparty/g2o/g2o/core/jacobian_workspace.cpp
  thirdparty/g2o/g2o/core/robust_kernel.cpp
  thirdparty/g2o/g2o/core/robust_kernel_factory.cpp
  thirdparty/g2o/g2o/core/robust_kernel_impl.cpp
  thirdparty/g2o/g2o/stuff/timeutil.cpp
  thirdparty/g2o/g2o/stuff/string_tools.cpp
  thirdparty/g2o/g2o/stuff/property.cpp)
target_include_directories(atlas_thirdparty_g2o SYSTEM PUBLIC
  "${ATLAS_THIRDPARTY}/g2o")
target_compile_definitions(atlas_thirdparty_g2o PRIVATE UNIX)
target_compile_features(atlas_thirdparty_g2o PUBLIC cxx_std_17)
target_link_libraries(atlas_thirdparty_g2o PUBLIC Eigen3::Eigen)

add_library(atlas_thirdparty_sophus INTERFACE)
target_include_directories(atlas_thirdparty_sophus SYSTEM INTERFACE
  "${ATLAS_THIRDPARTY}/Sophus")
target_link_libraries(atlas_thirdparty_sophus INTERFACE Eigen3::Eigen)

add_library(atlas_thirdparty_fbow INTERFACE)
target_include_directories(atlas_thirdparty_fbow SYSTEM INTERFACE
  "${ATLAS_THIRDPARTY}/FBow/include")
target_link_libraries(atlas_thirdparty_fbow INTERFACE ${ATLAS_FBOW_TARGET})

set(ATLAS_CORE_SOURCES
  atlas/common/time.cpp
  atlas/common/thread_pool.cpp
  atlas/common/blocking_queue.cpp
  atlas/common/config.cpp
  atlas/sensor/frame.cpp
  atlas/sensor/keyframe.cpp
  atlas/sensor/map_point.cpp
  atlas/sensor/imu_types.cpp
  atlas/sensor/camera/geometric_camera.cpp
  atlas/sensor/camera/pinhole_camera.cpp
  atlas/sensor/camera/kannala_brandt_camera.cpp
  atlas/sensor/camera/camera_factory.cpp
  atlas/feature/orb_extractor.cpp
  atlas/match/orb_matcher.cpp
  atlas/feature/vocabulary.cpp
  atlas/map_builder.cpp
  atlas/tracking/tracker.cpp
  atlas/tracking/tracking_state.cpp
  atlas/mapping/local_mapper.cpp
  atlas/closure/loop_closer.cpp
  atlas/closure/place_recognizer.cpp
  atlas/closure/pose_graph_optimizer.cpp
  atlas/map/atlas.cpp
  atlas/map/map.cpp
  atlas/optimization/optimizer.cpp
  atlas/optimization/g2o_types.cpp
  atlas/common/io/trajectory_io.cpp
  atlas/common/io/map_io.cpp
  atlas/common/io/binary_archive.cpp
  atlas/common/io/config_loader.cpp
  atlas/common/math/converter.cpp
  atlas/common/math/geometric_tools.cpp
  atlas/common/math/two_view_reconstruction.cpp
  atlas/match/ml_pnp_solver.cpp
  atlas/match/sim3_solver.cpp
  atlas/optimization/optimizable_types.cpp
  atlas/system.cpp
  atlas/frame_drawer.cpp
  atlas/map_drawer.cpp
  atlas/viewer.cpp)

add_library(atlas_core STATIC ${ATLAS_CORE_SOURCES})
target_include_directories(atlas_core PUBLIC
  "$<BUILD_INTERFACE:${ATLAS_ROOT}>"
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(atlas_core PUBLIC cxx_std_17)
target_compile_definitions(atlas_core PUBLIC COMPILEDWITHC11)
target_link_libraries(atlas_core PUBLIC
  atlas_thirdparty_g2o
  atlas_thirdparty_sophus
  atlas_thirdparty_fbow
  Eigen3::Eigen
  Threads::Threads
  OpenSSL::Crypto
  ${ATLAS_AUTOLINK_TARGET}
  ${ATLAS_OPENCV_TARGET})

if(ATLAS_ENABLE_OPENMP AND OpenMP_CXX_FOUND)
  target_compile_definitions(atlas_core PRIVATE
    ATLAS_USE_OPENMP EIGEN_DONT_PARALLELIZE)
  target_link_libraries(atlas_core PRIVATE OpenMP::OpenMP_CXX)
endif()

add_library(atlas_automsgs_bridge STATIC
  atlas/sensor/image_data.cpp
  atlas/sensor/imu_data.cpp
  atlas/sensor/pointcloud_data.cpp
  atlas/visual_slam_message_adapter.cpp)
target_include_directories(atlas_automsgs_bridge PUBLIC
  "$<BUILD_INTERFACE:${ATLAS_ROOT}>")
target_compile_features(atlas_automsgs_bridge PUBLIC cxx_std_17)
target_link_libraries(atlas_automsgs_bridge PUBLIC
  atlas_core
  ${ATLAS_AUTOMSGS_TARGET}
  ${ATLAS_AUTOLINK_TARGET}
  ${ATLAS_OPENCV_TARGET})

add_library(autonomy_localization_atlas STATIC
  atlas/atlas_node.cpp)
target_include_directories(autonomy_localization_atlas PUBLIC
  "$<BUILD_INTERFACE:${ATLAS_ROOT}>")
target_compile_features(autonomy_localization_atlas PUBLIC cxx_std_17)
target_link_libraries(autonomy_localization_atlas PUBLIC
  atlas_automsgs_bridge
  ${ATLAS_AUTOMSGS_TARGET}
  ${ATLAS_AUTOLINK_TARGET})

set_target_properties(
  atlas_thirdparty_g2o
  atlas_core
  atlas_automsgs_bridge
  autonomy_localization_atlas
  PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(autonomy::localization_atlas ALIAS autonomy_localization_atlas)

if(ATLAS_BUILD_EXAMPLES)
  add_library(atlas_dataset_readers STATIC
    examples/dataset_reader/euroc_reader.cpp
    examples/dataset_reader/tum_reader.cpp)
  target_include_directories(atlas_dataset_readers PUBLIC "${ATLAS_ROOT}")
  target_link_libraries(atlas_dataset_readers PUBLIC atlas_core)
  foreach(example IN ITEMS
      mono_euroc
      stereo_euroc
      rgbd_tum
      mono_inertial_euroc
      stereo_inertial_euroc)
    add_executable(atlas_${example} examples/${example}.cpp)
    target_link_libraries(atlas_${example} PRIVATE atlas_dataset_readers atlas_core)
  endforeach()
endif()

if(ATLAS_BUILD_TESTS)
  find_package(GTest REQUIRED)
  add_executable(atlas_unit_tests
    atlas/common/blocking_queue_test.cpp
    atlas/common/config_test.cpp
    atlas/sensor/camera/camera_factory_test.cpp
    atlas/feature/orb_extractor_test.cpp
    atlas/map_builder_test.cpp
    atlas/map/map_test.cpp
    atlas/common/io/config_loader_test.cpp)
  target_link_libraries(atlas_unit_tests PRIVATE atlas_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(atlas_unit_tests)
endif()

install(DIRECTORY config/
  DESTINATION share/autonomy/localization/conf/atlas
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.fbow")

