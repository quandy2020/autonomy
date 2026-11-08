/*
 * Copyright 2026 The Openbot Authors
 */
#include "map/serialization/session_writer.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "map/serialization/voxel_map_serializer.hpp"

namespace atlas {
namespace map {
namespace serialization {
namespace {

TEST(SessionWriter, WritesPoseAndScanCloudFiles) {
  const auto unique_suffix =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path base_path =
      std::filesystem::temp_directory_path() /
      ("atlas_session_writer_" + std::to_string(unique_suffix));
  const std::filesystem::path session_path = base_path / "session_a";

  SessionData session;
  session.name = "session_a";
  session.poses.resize(1);
  session.poses[0].x.t = 3.5;
  session.poses[0].x.p = Eigen::Vector3d(1.0, 2.0, 3.0);
  sensor::PointCloudXYZI cloud;
  sensor::PointType point;
  point.x = 1.0f;
  point.y = -2.0f;
  point.z = 0.5f;
  point.intensity = 7.0f;
  cloud.push_back(point);
  session.scan_clouds.push_back(cloud);

  std::string error;
  ASSERT_TRUE(SessionWriter().Write(base_path.string(), session, &error))
      << error;

  std::vector<SessionPoseEntry> loaded_poses;
  ASSERT_TRUE(LoadSessionPoses((session_path / "alidarState.txt").string(),
                               &loaded_poses));
  ASSERT_EQ(loaded_poses.size(), 1u);
  EXPECT_DOUBLE_EQ(loaded_poses[0].x.t, 3.5);
  EXPECT_NEAR(loaded_poses[0].x.p.y(), 2.0, 1e-6);

  sensor::PointCloudXYZI loaded_cloud;
  ASSERT_TRUE(LoadCloudBin((session_path / "0.bin").string(), &loaded_cloud));
  ASSERT_EQ(loaded_cloud.size(), 1u);
  EXPECT_FLOAT_EQ(loaded_cloud[0].intensity, 7.0f);

  std::error_code error_code;
  std::filesystem::remove_all(base_path, error_code);
}

TEST(SessionWriter, RejectsInconsistentOrUnsafeSessionData) {
  SessionData session;
  session.name = "../escape";
  session.poses.resize(1);
  session.scan_clouds.resize(1);

  std::string error;
  EXPECT_FALSE(SessionWriter().Write("/tmp", session, &error));
  EXPECT_FALSE(error.empty());

  session.name = "session_a";
  session.scan_clouds.clear();
  EXPECT_FALSE(SessionWriter().Write("/tmp", session, &error));
  EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace serialization
}  // namespace map
}  // namespace atlas
