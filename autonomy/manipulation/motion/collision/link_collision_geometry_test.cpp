/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include "autonomy/manipulation/motion/collision/link_collision_geometry.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {
namespace {

TEST(ConvexHullTest, BoxCornersProduceClosedHull) {
  std::vector<MeshVertex> verts = {
      {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
      {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
  };
  std::vector<MeshVertex> hv;
  std::vector<int> hf;
  ASSERT_TRUE(BuildConvexHull(verts, &hv, &hf));
  EXPECT_GE(hv.size(), 4u);
  EXPECT_GE(hf.size(), 12u);  // ≥ 4 triangles
  EXPECT_EQ(hf.size() % 3, 0u);
}

TEST(ConvexHullTest, FewPointsFallbackAabb) {
  std::vector<MeshVertex> verts = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  std::vector<MeshVertex> hv;
  std::vector<int> hf;
  ASSERT_TRUE(BuildConvexHull(verts, &hv, &hf));
  EXPECT_EQ(hv.size(), 8u);
  EXPECT_EQ(hf.size(), 36u);
}

}  // namespace
}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
