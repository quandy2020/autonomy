/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/pose_graph_committer.hpp"

#include <memory>
#include <vector>

#include "frontend/odometry/scan_pose.hpp"
#include "gtest/gtest.h"
#include "map/keyframe.hpp"

namespace atlas {
namespace backend {
namespace global_mapping {
namespace {

using ::atlas::frontend::odometry::ScanPose;
using ::atlas::frontend::odometry::ScanPosePtr;
using ::atlas::map::Keyframe;

TEST(PoseGraphCommitter, RejectsMismatchedLaterSessionWithoutPartialCommit) {
  auto first_source_scan = std::make_shared<ScanPose>();
  auto first_current_scan = std::make_shared<ScanPose>();
  auto second_source_scan = std::make_shared<ScanPose>();
  auto second_current_scan = std::make_shared<ScanPose>();
  first_source_scan->x.p.x() = 4.0;
  second_source_scan->x.p.x() = 9.0;
  Keyframe first_source_keyframe(first_source_scan->x);
  Keyframe first_current_keyframe(first_current_scan->x);
  Keyframe second_source_keyframe(second_source_scan->x);
  Keyframe second_current_keyframe(second_current_scan->x);
  first_source_keyframe.keyframe_id = first_current_keyframe.keyframe_id = 1;
  second_source_keyframe.keyframe_id = 2;
  second_current_keyframe.keyframe_id = 3;

  std::vector<ScanPosePtr> first_source_scans{first_source_scan};
  std::vector<ScanPosePtr> first_current_scans{first_current_scan};
  std::vector<ScanPosePtr> second_source_scans{second_source_scan};
  std::vector<ScanPosePtr> second_current_scans{second_current_scan};
  std::vector<Keyframe*> first_source_keyframes{&first_source_keyframe};
  std::vector<Keyframe*> first_current_keyframes{&first_current_keyframe};
  std::vector<Keyframe*> second_source_keyframes{&second_source_keyframe};
  std::vector<Keyframe*> second_current_keyframes{&second_current_keyframe};
  std::vector<std::vector<ScanPosePtr>*> optimized_scans{&first_source_scans,
                                                         &second_source_scans};
  std::vector<std::vector<Keyframe*>*> optimized_keyframes{
      &first_source_keyframes, &second_source_keyframes};
  std::vector<std::vector<ScanPosePtr>*> current_scans{&first_current_scans,
                                                       &second_current_scans};
  std::vector<std::vector<Keyframe*>*> current_keyframes{
      &first_current_keyframes, &second_current_keyframes};

  PoseGraphCommitter committer;
  EXPECT_FALSE(committer.Commit(optimized_scans, optimized_keyframes,
                                current_scans, current_keyframes));
  EXPECT_DOUBLE_EQ(first_current_scan->x.p.x(), 0.0);
  EXPECT_DOUBLE_EQ(second_current_scan->x.p.x(), 0.0);
}

TEST(PoseGraphCommitter, CommitsAllValidatedSessionPoses) {
  auto optimized_scan = std::make_shared<ScanPose>();
  auto current_scan = std::make_shared<ScanPose>();
  optimized_scan->x.p.x() = 3.0;
  Keyframe optimized_keyframe(optimized_scan->x);
  Keyframe current_keyframe(current_scan->x);
  optimized_keyframe.keyframe_id = current_keyframe.keyframe_id = 7;
  optimized_keyframe.session_id = current_keyframe.session_id = 2;

  std::vector<ScanPosePtr> optimized_scan_values{optimized_scan};
  std::vector<ScanPosePtr> current_scan_values{current_scan};
  std::vector<Keyframe*> optimized_keyframe_values{&optimized_keyframe};
  std::vector<Keyframe*> current_keyframe_values{&current_keyframe};
  std::vector<std::vector<ScanPosePtr>*> optimized_scans{
      &optimized_scan_values};
  std::vector<std::vector<Keyframe*>*> optimized_keyframes{
      &optimized_keyframe_values};
  std::vector<std::vector<ScanPosePtr>*> current_scans{&current_scan_values};
  std::vector<std::vector<Keyframe*>*> current_keyframes{
      &current_keyframe_values};

  PoseGraphCommitter committer;
  ASSERT_TRUE(committer.Commit(optimized_scans, optimized_keyframes,
                               current_scans, current_keyframes));
  EXPECT_DOUBLE_EQ(current_scan->x.p.x(), 3.0);
  EXPECT_DOUBLE_EQ(current_keyframe.inertial_state.p.x(), 3.0);
}

}  // namespace
}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas
