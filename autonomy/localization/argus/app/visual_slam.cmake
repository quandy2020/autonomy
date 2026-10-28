find_package(ZLIB REQUIRED)

add_library(argus_visual_slam STATIC
  orb_vocabulary_cache.cpp
  visual_slam_system.cpp
)
target_include_directories(argus_visual_slam
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ARGUS_LIBRARY_ROOT}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(argus_visual_slam
  PUBLIC
    argus::visual_orb
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ZLIB::ZLIB
)
target_compile_features(argus_visual_slam PUBLIC cxx_std_17)
set_target_properties(argus_visual_slam PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(argus::visual_slam ALIAS argus_visual_slam)
