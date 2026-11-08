/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "frontend/odometry/point_covariance_estimator.hpp"

#include <gtest/gtest.h>

#include "frontend/odometry/downsample.hpp"
#include "map/point_observation_transform.hpp"

namespace atlas {
namespace frontend {
namespace odometry {

using ::atlas::frontend::inertial::InertialState;
using ::atlas::map::WorldPointList;
namespace {

sensor::PointCloudXYZI MakeCloud(int size) {
  sensor::PointCloudXYZI cloud;
  for (int i = 0; i < size; ++i) {
    sensor::PointType point;
    point.x = 0.1f * i;
    point.y = 0.f;
    point.z = 1.f;
    point.curvature = 0.01f * i;
    cloud.push_back(point);
  }
  return cloud;
}

TEST(PointCovariance, BodyPointCovarianceIsPositiveDefinite) {
  Eigen::Matrix3d covariance;
  PointCovarianceEstimator::CalculateBodyPointCovariance(
      Eigen::Vector3d(1.0, 0.0, 2.0), 0.05f, 0.2f, &covariance);
  EXPECT_GT(covariance.norm(), 1e-9);
  EXPECT_GT(covariance.determinant(), 0.0);
}

TEST(PointCovariance, InitializationAndWorldTransformKeepSize) {
  const sensor::PointCloudXYZI cloud = MakeCloud(5);
  InertialState extrinsic;
  PointObservationList observations;
  PointCovarianceEstimator::InitializePointObservations(extrinsic, cloud, 0.05,
                                                        0.2, &observations);
  ASSERT_EQ(observations.size(), 5u);

  InertialState state;
  state.p = Eigen::Vector3d(1.0, 2.0, 3.0);
  WorldPointList world;
  TransformPointObservationsToWorld(
      common::PoseState(state.t, state.R, state.p, state.cov.block<6, 6>(0, 0)),
      &observations, &world);
  ASSERT_EQ(world.size(), 5u);
  EXPECT_NEAR(world[0].z(), 4.0, 1e-9);
}

TEST(Downsample, VoxelAverageCollapsesPoints) {
  sensor::PointCloudXYZI cloud = MakeCloud(5);
  DownsampleVoxel(&cloud, 1.0);
  EXPECT_EQ(cloud.size(), 1u);
  EXPECT_NEAR(cloud[0].x, 0.2f, 1e-6);
}

}  // namespace
}  // namespace odometry
}  // namespace frontend
}  // namespace atlas
