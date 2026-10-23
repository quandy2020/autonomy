cmake_minimum_required(VERSION 3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(voxel_slam_system LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  include(CTest)
endif()

set(ATLAS_LIBRARY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../atlas)

find_package(Eigen3 REQUIRED)

add_library(voxel_slam_system
  slam_system_config.cpp
  slam_system_builder.cpp
  slam_system.cpp
  voxel_slam_system.cpp
)
target_include_directories(voxel_slam_system
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ATLAS_LIBRARY_ROOT}
)
target_link_libraries(voxel_slam_system
  PUBLIC
    Eigen3::Eigen
    atlas_voxel
)
target_compile_features(voxel_slam_system PUBLIC cxx_std_17)

if(ATLAS_BUILD_MESSAGE_ADAPTERS)
  add_library(voxel_slam_message_adapter STATIC
    voxel_slam_message_adapter.cpp
  )
  target_include_directories(voxel_slam_message_adapter
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${ATLAS_LIBRARY_ROOT}
  )
  target_link_libraries(voxel_slam_message_adapter
    PUBLIC
      voxel_slam_system
      atlas_automsgs
      ${ATLAS_AUTOMSGS_TARGET}
  )
  target_compile_features(voxel_slam_message_adapter PUBLIC cxx_std_17)
endif()

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)

  function(voxel_slam_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE voxel_slam_system GTest::gtest_main)
    gtest_discover_tests(${name})
  endfunction()

  voxel_slam_test(slam_system_config_test slam_system_config_test.cpp)
  voxel_slam_test(slam_system_builder_test slam_system_builder_test.cpp)
  voxel_slam_test(slam_system_test slam_system_test.cpp)
  voxel_slam_test(voxel_slam_system_test voxel_slam_system_test.cpp)
  voxel_slam_test(factory_test ${ATLAS_LIBRARY_ROOT}/common/factory_test.cpp)
endif()

add_executable(run_slam run_slam_main.cpp)
add_executable(replay replay_main.cpp)
add_executable(print_registry print_registry_main.cpp)
target_link_libraries(run_slam PRIVATE voxel_slam_system)
target_link_libraries(replay PRIVATE voxel_slam_system)
target_link_libraries(print_registry PRIVATE voxel_slam_system)

if(BUILD_TESTING)
  add_test(NAME print_registry COMMAND print_registry)
endif()
