/*
 * Copyright 2026 The Openbot Authors
 */

#include "backend/pose_graph/loop_pose_graph.hpp"

#include <unordered_map>

#include "gtest/gtest.h"

namespace atlas {
namespace backend {
namespace pose_graph {
namespace {

transform::Rigid3d MakePose(double x) {
  return transform::Rigid3d(Eigen::Vector3d(x, 0.0, 0.0),
                            Eigen::Quaterniond::Identity());
}

TEST(LoopPoseGraph, RejectsStalePoseCommitWithoutChangingCurrentState) {
  LoopPoseGraph graph;
  graph.AddNode(0, MakePose(1.0));
  const LoopPoseGraphSnapshot source_snapshot = graph.Snapshot();
  graph.SetPose(0, MakePose(2.0));
  const LoopPoseGraphSnapshot current_snapshot = graph.Snapshot();

  EXPECT_FALSE(graph.CommitPoses(source_snapshot.state_version,
                                 source_snapshot.poses, 0.0));

  const LoopPoseGraphSnapshot result_snapshot = graph.Snapshot();
  EXPECT_EQ(result_snapshot.state_version, current_snapshot.state_version);
  EXPECT_DOUBLE_EQ(result_snapshot.poses.at(0).translation().x(), 2.0);
}

TEST(LoopPoseGraph, CommitsPoseSetForCurrentVersionAndAdvancesVersion) {
  LoopPoseGraph graph;
  graph.AddNode(0, MakePose(1.0));
  const LoopPoseGraphSnapshot source_snapshot = graph.Snapshot();
  std::unordered_map<int, transform::Rigid3d> optimized_poses =
      source_snapshot.poses;
  optimized_poses.at(0) = MakePose(3.0);

  EXPECT_TRUE(
      graph.CommitPoses(source_snapshot.state_version, optimized_poses, 0.5));

  const LoopPoseGraphSnapshot result_snapshot = graph.Snapshot();
  EXPECT_EQ(result_snapshot.state_version, source_snapshot.state_version + 1);
  EXPECT_DOUBLE_EQ(result_snapshot.poses.at(0).translation().x(), 3.0);
}

}  // namespace
}  // namespace pose_graph
}  // namespace backend
}  // namespace atlas
