/*
 * Copyright 2026 The Openbot Authors
 */

#include "map/point_observation_transform.hpp"

#include <gtest/gtest.h>

#include "common/pose_state.hpp"

namespace atlas {
namespace map {
namespace {

TEST(PointObservationTransform, AppliesPoseAndPoseCovariance) {
  common::PoseState pose;
  pose.translation = Eigen::Vector3d(1.0, 2.0, 3.0);
  pose.covariance.setZero();
  pose.covariance.block<3, 3>(0, 0).diagonal().setConstant(0.01);
  pose.covariance.block<3, 3>(3, 3).diagonal().setConstant(0.04);

  PointObservationList observations(1);
  observations.front().position = Eigen::Vector3d(0.0, 0.0, 2.0);
  observations.front().covariance.setIdentity();
  WorldPointList world_points;

  TransformPointObservationsToWorld(pose, &observations, &world_points);

  ASSERT_EQ(world_points.size(), 1u);
  EXPECT_TRUE(world_points.front().isApprox(Eigen::Vector3d(1.0, 2.0, 5.0)));
  Eigen::Matrix3d expected_covariance = Eigen::Matrix3d::Identity();
  expected_covariance.diagonal().array() += 0.04;
  expected_covariance(0, 0) += 0.04;
  expected_covariance(1, 1) += 0.04;
  EXPECT_TRUE(observations.front().covariance.isApprox(expected_covariance));
}

}  // namespace
}  // namespace map
}  // namespace atlas
