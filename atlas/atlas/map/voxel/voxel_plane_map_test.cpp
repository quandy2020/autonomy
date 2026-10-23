#include "map/voxel/voxel_plane_map.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace atlas {
namespace map {
namespace voxel {
namespace {

std::vector<Eigen::Vector3d> Grid(const Eigen::Vector3d& origin,
                                  const Eigen::Vector3d& axis_u,
                                  const Eigen::Vector3d& axis_v) {
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 4; ++j) {
      points.push_back(origin + axis_u * (0.08 * i) + axis_v * (0.08 * j));
    }
  }
  return points;
}

TEST(VoxelPlaneMap, FitsAPlaneInsideOneVoxel) {
  VoxelPlaneMap map;
  for (const Eigen::Vector3d& point :
       Grid(Eigen::Vector3d(0.3, 0.3, 0.5), Eigen::Vector3d::UnitX(),
            Eigen::Vector3d::UnitY())) {
    map.Insert(point);
  }
  EXPECT_EQ(map.plane_count(), 0u);
  map.Commit();
  ASSERT_EQ(map.plane_count(), 1u);
  Eigen::Vector3d center;
  Eigen::Vector3d normal;
  ASSERT_TRUE(map.Query(Eigen::Vector3d(0.46, 0.42, 0.55), &center, &normal));
  EXPECT_NEAR(std::abs(normal.z()), 1.0, 1e-9);
  EXPECT_NEAR(center.z(), 0.5, 1e-9);
  EXPECT_FALSE(map.Query(Eigen::Vector3d(0.46, 0.42, 0.8), &center, &normal));
}

TEST(VoxelPlaneMap, RejectsAFilledVoxel) {
  VoxelPlaneMap::Config config;
  config.min_eigenvalue = 1e-4;
  VoxelPlaneMap map(config);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        map.Insert(
            Eigen::Vector3d(0.2 + 0.2 * i, 0.2 + 0.2 * j, 0.2 + 0.2 * k));
      }
    }
  }
  map.Commit();
  EXPECT_EQ(map.plane_count(), 0u);
  Eigen::Vector3d center;
  Eigen::Vector3d normal;
  EXPECT_FALSE(map.Query(Eigen::Vector3d(0.4, 0.4, 0.4), &center, &normal));
}

TEST(VoxelPlaneMap, SplitsTwoPlanesThatShareAVoxel) {
  VoxelPlaneMap::Config config;
  config.min_eigenvalue = 1e-3;
  VoxelPlaneMap map(config);
  for (const Eigen::Vector3d& point :
       Grid(Eigen::Vector3d(0.1, 0.1, 0.2), Eigen::Vector3d::UnitX(),
            Eigen::Vector3d::UnitY())) {
    map.Insert(point);
  }
  for (const Eigen::Vector3d& point :
       Grid(Eigen::Vector3d(0.8, 0.6, 0.6), Eigen::Vector3d::UnitY(),
            Eigen::Vector3d::UnitZ())) {
    map.Insert(point);
  }
  map.Commit();
  EXPECT_EQ(map.plane_count(), 2u);
  Eigen::Vector3d center;
  Eigen::Vector3d normal;
  ASSERT_TRUE(map.Query(Eigen::Vector3d(0.26, 0.22, 0.24), &center, &normal));
  EXPECT_NEAR(std::abs(normal.z()), 1.0, 1e-6);
  ASSERT_TRUE(map.Query(Eigen::Vector3d(0.84, 0.72, 0.72), &center, &normal));
  EXPECT_NEAR(std::abs(normal.x()), 1.0, 1e-6);
}

TEST(VoxelPlaneMap, IndexesNegativeCoordinates) {
  VoxelPlaneMap map;
  for (const Eigen::Vector3d& point :
       Grid(Eigen::Vector3d(-0.7, 0.3, 0.3), Eigen::Vector3d::UnitX(),
            Eigen::Vector3d::UnitY())) {
    map.Insert(point);
  }
  map.Commit();
  Eigen::Vector3d center;
  Eigen::Vector3d normal;
  ASSERT_TRUE(map.Query(Eigen::Vector3d(-0.54, 0.42, 0.34), &center, &normal));
  EXPECT_NEAR(center.z(), 0.3, 1e-9);
}

}  // namespace
}  // namespace voxel
}  // namespace map
}  // namespace atlas
