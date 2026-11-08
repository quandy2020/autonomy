/*
 * Copyright 2026 The Openbot Authors
 */
#include "map/serialization/session_reader.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "map/serialization/session_writer.hpp"

namespace atlas {
namespace map {
namespace serialization {
namespace {

std::filesystem::path MakeTemporaryDirectoryPath() {
  const auto suffix =
      std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("atlas_session_reader_" + std::to_string(suffix));
}

TEST(SessionReader, ReadsPosesAndCloudsByScanIndex) {
  const std::filesystem::path base_path = MakeTemporaryDirectoryPath();
  SessionData session;
  session.name = "session_b";
  session.poses.resize(2);
  session.poses[0].x.t = 1.0;
  session.poses[1].x.t = 2.0;
  session.poses[1].x.p = Eigen::Vector3d(4.0, 5.0, 6.0);
  session.scan_clouds.resize(2);
  sensor::PointType expected_point;
  expected_point.x = 3.0f;
  expected_point.y = -1.0f;
  expected_point.z = 0.25f;
  expected_point.intensity = 9.0f;
  session.scan_clouds[1].push_back(expected_point);

  std::string error;
  ASSERT_TRUE(SessionWriter().Write(base_path.string(), session, &error))
      << error;

  SessionReader reader;
  std::vector<SessionPoseEntry> poses;
  ASSERT_TRUE(
      reader.ReadPoses(base_path.string(), session.name, &poses, &error))
      << error;
  ASSERT_EQ(poses.size(), 2u);
  EXPECT_DOUBLE_EQ(poses[1].x.t, 2.0);
  EXPECT_NEAR(poses[1].x.p.z(), 6.0, 1e-6);

  sensor::PointCloudXYZI cloud;
  ASSERT_TRUE(
      reader.ReadCloud(base_path.string(), session.name, 1, &cloud, &error))
      << error;
  ASSERT_EQ(cloud.size(), 1u);
  EXPECT_FLOAT_EQ(cloud[0].intensity, 9.0f);

  std::error_code error_code;
  std::filesystem::remove_all(base_path, error_code);
}

TEST(SessionReader, FailedReadsLeaveOutputsUnchanged) {
  SessionReader reader;
  std::vector<SessionPoseEntry> poses(1);
  poses[0].x.t = 12.0;
  sensor::PointCloudXYZI cloud;
  sensor::PointType point;
  point.x = 4.0f;
  cloud.push_back(point);
  std::string error;

  EXPECT_FALSE(reader.ReadPoses("/tmp", "../outside", &poses, &error));
  ASSERT_EQ(poses.size(), 1u);
  EXPECT_DOUBLE_EQ(poses[0].x.t, 12.0);

  EXPECT_FALSE(reader.ReadCloud("/tmp", "missing_session", 0, &cloud, &error));
  ASSERT_EQ(cloud.size(), 1u);
  EXPECT_FLOAT_EQ(cloud[0].x, 4.0f);
}

}  // namespace
}  // namespace serialization
}  // namespace map
}  // namespace atlas
