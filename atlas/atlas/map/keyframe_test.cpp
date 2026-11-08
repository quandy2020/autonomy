/*
 * Copyright 2026 The Openbot Authors
 */

#include "map/keyframe.hpp"

#include <gtest/gtest.h>

#include "frontend/inertial/inertial_state.hpp"

namespace atlas {
namespace map {
namespace {

TEST(Keyframe, StoresInitialInertialStateAndPointCloud) {
  frontend::inertial::InertialState initial_state;
  initial_state.t = 2.5;
  initial_state.p = Eigen::Vector3d(1.0, 2.0, 3.0);
  Keyframe keyframe(initial_state);

  keyframe.keyframe_id = 7;
  keyframe.session_id = 2;
  keyframe.map_insertion_pending = true;
  keyframe.trajectory_distance_meters = 4.5;

  EXPECT_EQ(keyframe.keyframe_id, 7);
  EXPECT_EQ(keyframe.session_id, 2);
  EXPECT_TRUE(keyframe.map_insertion_pending);
  EXPECT_DOUBLE_EQ(keyframe.trajectory_distance_meters, 4.5);
  EXPECT_DOUBLE_EQ(keyframe.inertial_state.t, 2.5);
  EXPECT_TRUE(keyframe.inertial_state.p.isApprox(initial_state.p));
  ASSERT_NE(keyframe.point_cloud, nullptr);
  EXPECT_TRUE(keyframe.point_cloud->empty());
}

}  // namespace
}  // namespace map
}  // namespace atlas
