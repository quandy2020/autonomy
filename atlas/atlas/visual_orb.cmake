# ORB-SLAM3 visual and visual-inertial pipeline migrated into Atlas.
# Upstream algorithm code remains GPLv3 and is isolated behind Atlas APIs.

find_package(OpenCV 4 REQUIRED
  COMPONENTS core features2d imgproc imgcodecs calib3d)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/orb_slam3
  ${CMAKE_CURRENT_BINARY_DIR}/orb_slam3_third_party
)

add_library(atlas_visual_orb STATIC
  frontend/visual/orb_slam3/src/System.cc
  frontend/visual/orb_slam3/src/Tracking.cc
  frontend/visual/orb_slam3/src/LocalMapping.cc
  frontend/visual/orb_slam3/src/LoopClosing.cc
  frontend/visual/orb_slam3/src/ORBextractor.cc
  frontend/visual/orb_slam3/src/ORBmatcher.cc
  frontend/visual/orb_slam3/src/FrameDrawer.cc
  frontend/visual/orb_slam3/src/Converter.cc
  frontend/visual/orb_slam3/src/MapPoint.cc
  frontend/visual/orb_slam3/src/KeyFrame.cc
  frontend/visual/orb_slam3/src/Atlas.cc
  frontend/visual/orb_slam3/src/Map.cc
  frontend/visual/orb_slam3/src/MapDrawerHeadless.cc
  frontend/visual/orb_slam3/src/Optimizer.cc
  frontend/visual/orb_slam3/src/Frame.cc
  frontend/visual/orb_slam3/src/KeyFrameDatabase.cc
  frontend/visual/orb_slam3/src/Sim3Solver.cc
  frontend/visual/orb_slam3/src/ViewerHeadless.cc
  frontend/visual/orb_slam3/src/ImuTypes.cc
  frontend/visual/orb_slam3/src/G2oTypes.cc
  frontend/visual/orb_slam3/src/CameraModels/Pinhole.cpp
  frontend/visual/orb_slam3/src/CameraModels/KannalaBrandt8.cpp
  frontend/visual/orb_slam3/src/OptimizableTypes.cpp
  frontend/visual/orb_slam3/src/MLPnPsolver.cpp
  frontend/visual/orb_slam3/src/GeometricTools.cc
  frontend/visual/orb_slam3/src/TwoViewReconstruction.cc
  frontend/visual/orb_slam3/src/Config.cc
  frontend/visual/orb_slam3/src/Settings.cc
)

target_include_directories(atlas_visual_orb
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/frontend/visual/orb_slam3
    ${CMAKE_CURRENT_SOURCE_DIR}/frontend/visual/orb_slam3/include
    ${CMAKE_CURRENT_SOURCE_DIR}/frontend/visual/orb_slam3/include/CameraModels
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(atlas_visual_orb
  PUBLIC
    Eigen3::Eigen
    Threads::Threads
    atlas::orb_sophus
  PRIVATE
    atlas::orb_dbow2
    atlas::orb_g2o
    Boost::serialization
    OpenSSL::Crypto
    ${OpenCV_LIBS}
)

target_compile_definitions(atlas_visual_orb PRIVATE COMPILEDWITHC11)
target_compile_features(atlas_visual_orb PUBLIC cxx_std_17)
set_target_properties(
  atlas_visual_orb atlas_orb_dbow2 atlas_orb_g2o
  PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(atlas::visual_orb ALIAS atlas_visual_orb)
