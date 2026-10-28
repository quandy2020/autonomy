find_package(ZLIB REQUIRED)

add_library(atlas_visual_slam STATIC
  orb_vocabulary_cache.cpp
  visual_slam_system.cpp
)
target_include_directories(atlas_visual_slam
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ATLAS_LIBRARY_ROOT}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(atlas_visual_slam
  PUBLIC
    atlas::visual_orb
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ZLIB::ZLIB
)
target_compile_features(atlas_visual_slam PUBLIC cxx_std_17)
set_target_properties(atlas_visual_slam PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(atlas::visual_slam ALIAS atlas_visual_slam)
