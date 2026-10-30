cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(autonomy_localization_atlas LANGUAGES CXX)
endif()

set(ATLAS_ROOT "${CMAKE_CURRENT_LIST_DIR}")

include("${ATLAS_ROOT}/cmake/functions.cmake")
atlas_initialize_project()

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
  find_package(OpenCV REQUIRED COMPONENTS
    core
    features2d
    imgproc
    imgcodecs
    calib3d)
endif()

if(NOT TARGET OpenSSL::Crypto)
  find_package(OpenSSL REQUIRED COMPONENTS Crypto)
endif()

if(ATLAS_ENABLE_OPENMP)
  find_package(OpenMP QUIET COMPONENTS CXX)
  if(NOT OpenMP_CXX_FOUND)
    message(WARNING
      "Atlas OpenMP support requested but unavailable; "
      "using the thread-pool fallback")
  endif()
endif()

find_package(g2o 1.0 CONFIG REQUIRED)
find_package(fbow CONFIG REQUIRED)

if(NOT TARGET fbow::fbow)
  message(FATAL_ERROR "Atlas requires the system fbow::fbow target")
endif()

# The installed fbow package advertises OpenCV libraries from its package build.
# Atlas selects the parent project OpenCV target explicitly.
set_property(TARGET fbow::fbow PROPERTY INTERFACE_LINK_LIBRARIES "")

if(NOT TARGET automsgs::automsgs AND NOT TARGET automsgs::core AND
   NOT TARGET automsgs)
  find_package(automsgs CONFIG QUIET)
endif()
if(NOT TARGET automsgs::automsgs AND NOT TARGET automsgs::core AND
   NOT TARGET automsgs)
  find_package(AutoMsgs REQUIRED MODULE)
endif()

if(TARGET automsgs::automsgs)
  set(ATLAS_AUTOMSGS_TARGET automsgs::automsgs)
elseif(TARGET automsgs::core)
  set(ATLAS_AUTOMSGS_TARGET automsgs::core)
elseif(TARGET automsgs)
  set(ATLAS_AUTOMSGS_TARGET automsgs)
else()
  message(FATAL_ERROR "Atlas requires an AutoMsgs target")
endif()

if(NOT TARGET Autolink::Autolink AND NOT TARGET autolink::autolink AND
   NOT TARGET autolink)
  find_package(Autolink CONFIG QUIET)
endif()
if(NOT TARGET Autolink::Autolink AND NOT TARGET autolink::autolink AND
   NOT TARGET autolink)
  find_package(Autolink REQUIRED MODULE)
endif()

if(TARGET Autolink::Autolink)
  set(ATLAS_AUTOLINK_TARGET Autolink::Autolink)
elseif(TARGET autolink::autolink)
  set(ATLAS_AUTOLINK_TARGET autolink::autolink)
elseif(TARGET autolink)
  set(ATLAS_AUTOLINK_TARGET autolink)
else()
  message(FATAL_ERROR "Atlas requires an Autolink target")
endif()

if(TARGET autonomy_opencv)
  set(ATLAS_OPENCV_TARGET autonomy_opencv)
else()
  set(ATLAS_OPENCV_TARGET ${OpenCV_LIBS})
endif()

file(GLOB_RECURSE ATLAS_ALL_HEADERS CONFIGURE_DEPENDS
  "${ATLAS_ROOT}/atlas/*.hpp")
file(GLOB_RECURSE ATLAS_ALL_SOURCES CONFIGURE_DEPENDS
  "${ATLAS_ROOT}/atlas/*.cpp")
file(GLOB_RECURSE ATLAS_TEST_SOURCES CONFIGURE_DEPENDS
  "${ATLAS_ROOT}/atlas/*_test.cpp")
set(ATLAS_NODE_TEST_SOURCE
  "${ATLAS_ROOT}/atlas/atlas_node_test.cpp")
list(REMOVE_ITEM ATLAS_TEST_SOURCES
  ${ATLAS_NODE_TEST_SOURCE})

set(ATLAS_AUTOMSGS_BRIDGE_SOURCES
  "${ATLAS_ROOT}/atlas/sensor/image_data.cpp"
  "${ATLAS_ROOT}/atlas/sensor/imu_data.cpp"
  "${ATLAS_ROOT}/atlas/sensor/pointcloud_data.cpp"
  "${ATLAS_ROOT}/atlas/message_adapter.cpp")
set(ATLAS_NODE_SOURCES
  "${ATLAS_ROOT}/atlas/atlas_node.cpp")

set(ATLAS_CORE_SOURCES ${ATLAS_ALL_SOURCES})
list(REMOVE_ITEM ATLAS_CORE_SOURCES
  ${ATLAS_TEST_SOURCES}
  ${ATLAS_NODE_TEST_SOURCE}
  ${ATLAS_AUTOMSGS_BRIDGE_SOURCES}
  ${ATLAS_NODE_SOURCES})


add_library(atlas_core STATIC
  ${ATLAS_ALL_HEADERS}
  ${ATLAS_CORE_SOURCES})
atlas_configure_target(atlas_core)
target_include_directories(atlas_core PUBLIC
  "$<BUILD_INTERFACE:${ATLAS_ROOT}>"
  "../../.."
  ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(atlas_core PUBLIC COMPILEDWITHC11)
target_link_libraries(atlas_core PUBLIC
  g2o::core
  g2o::stuff
  g2o::types_sba
  g2o::types_sim3
  g2o::solver_dense
  g2o::solver_eigen
  fbow::fbow
  Eigen3::Eigen
  Threads::Threads
  OpenSSL::Crypto
  ${ATLAS_AUTOLINK_TARGET}
  ${ATLAS_OPENCV_TARGET})

if(ATLAS_ENABLE_OPENMP AND OpenMP_CXX_FOUND)
  target_compile_definitions(atlas_core PRIVATE
    ATLAS_USE_OPENMP
    EIGEN_DONT_PARALLELIZE)
  target_link_libraries(atlas_core PRIVATE OpenMP::OpenMP_CXX)
endif()

add_library(atlas_automsgs_bridge STATIC
  ${ATLAS_AUTOMSGS_BRIDGE_SOURCES})
atlas_configure_target(atlas_automsgs_bridge)
target_include_directories(atlas_automsgs_bridge PUBLIC
  "$<BUILD_INTERFACE:${ATLAS_ROOT}>")
target_link_libraries(atlas_automsgs_bridge PUBLIC
  atlas_core
  ${ATLAS_AUTOMSGS_TARGET}
  ${ATLAS_AUTOLINK_TARGET}
  ${ATLAS_OPENCV_TARGET})

add_library(autonomy_localization_atlas STATIC
  ${ATLAS_NODE_SOURCES})
atlas_configure_target(autonomy_localization_atlas)
target_include_directories(autonomy_localization_atlas PUBLIC
  "$<BUILD_INTERFACE:${ATLAS_ROOT}>")
target_link_libraries(autonomy_localization_atlas PUBLIC
  atlas_automsgs_bridge
  autonomy_transform
  ${ATLAS_AUTOMSGS_TARGET}
  ${ATLAS_AUTOLINK_TARGET})

add_library(
  autonomy::localization_atlas
  ALIAS autonomy_localization_atlas)

if(ATLAS_BUILD_EXAMPLES OR ATLAS_BUILD_TESTS)
  file(GLOB ATLAS_DATASET_READER_SOURCES CONFIGURE_DEPENDS
    "${ATLAS_ROOT}/examples/dataset_reader/*.cpp")
  file(GLOB ATLAS_DATASET_READER_TEST_SOURCES CONFIGURE_DEPENDS
    "${ATLAS_ROOT}/examples/dataset_reader/*_test.cpp")
  list(REMOVE_ITEM ATLAS_DATASET_READER_SOURCES
    ${ATLAS_DATASET_READER_TEST_SOURCES})
  add_library(atlas_dataset_readers STATIC
    ${ATLAS_DATASET_READER_SOURCES})
  atlas_configure_target(atlas_dataset_readers)
  target_include_directories(atlas_dataset_readers PUBLIC
    "${ATLAS_ROOT}")
  target_link_libraries(atlas_dataset_readers PUBLIC atlas_core)
endif()

if(ATLAS_BUILD_EXAMPLES)
  foreach(example IN ITEMS
      mono_euroc
      stereo_euroc
      rgbd_tum
      rgbd_inertial_tartanair
      mono_inertial_euroc
      stereo_inertial_euroc
      stereo_inertial_tum_vi
      stereo_kitti)
    atlas_binary(atlas_${example}
      SRCS "${ATLAS_ROOT}/examples/${example}.cpp"
      LIBRARIES atlas_dataset_readers)
  endforeach()
endif()

if(ATLAS_BUILD_TESTS)
  atlas_enable_testing()
  atlas_test(atlas_unit_tests
    SRCS ${ATLAS_TEST_SOURCES})
  atlas_test(atlas_node_tests
    SRCS ${ATLAS_NODE_TEST_SOURCE}
    LIBRARIES autonomy_localization_atlas)
  if(ATLAS_DATASET_READER_TEST_SOURCES)
    atlas_test(atlas_dataset_reader_tests
      SRCS ${ATLAS_DATASET_READER_TEST_SOURCES}
      LIBRARIES atlas_dataset_readers)
  endif()
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_Interpreter_FOUND)
    add_test(
      NAME atlas_evaluate_euroc_test
      COMMAND ${Python3_EXECUTABLE}
        "${ATLAS_ROOT}/examples/evaluate_euroc_test.py")
    add_test(
      NAME atlas_evaluate_tartanair_test
      COMMAND ${Python3_EXECUTABLE}
        "${ATLAS_ROOT}/examples/evaluate_tartanair_test.py")
    add_test(
      NAME atlas_evaluate_tum_rgbd_test
      COMMAND ${Python3_EXECUTABLE}
        "${ATLAS_ROOT}/examples/evaluate_tum_rgbd_test.py")
    add_test(
      NAME atlas_prepare_tum_rgbd_test
      COMMAND ${Python3_EXECUTABLE}
        "${ATLAS_ROOT}/examples/prepare_tum_rgbd_test.py")
    add_test(
      NAME atlas_run_all_benchmarks_test
      COMMAND ${Python3_EXECUTABLE}
        "${ATLAS_ROOT}/examples/run_all_benchmarks_test.py")
  endif()
endif()

install(DIRECTORY "${ATLAS_ROOT}/config/"
  DESTINATION share/autonomy/localization/conf/atlas
  FILES_MATCHING
    PATTERN "*.yaml"
    PATTERN "*.fbow")

install(DIRECTORY "${ATLAS_ROOT}/cmake/"
  DESTINATION share/autonomy/localization/atlas/cmake)
