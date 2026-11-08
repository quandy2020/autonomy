/*
 * Copyright 2026 The Openbot Authors
 */
#include "map/serialization/session_writer.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <system_error>

#include "map/serialization/voxel_map_serializer.hpp"

namespace atlas {
namespace map {
namespace serialization {

bool SessionWriter::Write(const std::string& base_path,
                          const SessionData& session,
                          std::string* error) const {
  const auto fail = [error](const std::string& message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };
  if (session.name.empty() || session.poses.empty() ||
      session.poses.size() != session.scan_clouds.size()) {
    return fail("session name, poses, and scan clouds are inconsistent");
  }

  namespace fs = std::filesystem;
  const fs::path session_name(session.name);
  if (session_name.is_absolute() || session_name.has_parent_path() ||
      session.name == "." || session.name == "..") {
    return fail("session name must be a single directory name");
  }
  const fs::path session_path = fs::path(base_path) / session_name;
  std::error_code filesystem_error;
  fs::create_directories(session_path, filesystem_error);
  if (filesystem_error) {
    return fail("cannot create session directory " + session_path.string() +
                ": " + filesystem_error.message());
  }

  for (const SessionPoseEntry& pose : session.poses) {
    if (!std::isfinite(pose.x.t) || !pose.x.p.allFinite() ||
        !pose.x.R.allFinite() || !pose.x.v.allFinite() ||
        !pose.x.bg.allFinite() || !pose.x.ba.allFinite() ||
        !pose.x.g.allFinite() || !pose.v6.allFinite()) {
      return fail("session contains an invalid scan pose");
    }
  }
  for (const sensor::PointCloudXYZI& cloud : session.scan_clouds) {
    if (std::any_of(cloud.points.begin(), cloud.points.end(),
                    [](const sensor::PointType& point) {
                      return !std::isfinite(point.x) ||
                             !std::isfinite(point.y) ||
                             !std::isfinite(point.z) ||
                             !std::isfinite(point.intensity);
                    })) {
      return fail("session contains an invalid scan cloud");
    }
  }

  const fs::path pose_path = session_path / "alidarState.txt";
  if (!SaveSessionPoses(pose_path.string(), session.poses)) {
    return fail("cannot save session poses under " + session_path.string());
  }
  for (std::size_t index = 0; index < session.scan_clouds.size(); ++index) {
    const fs::path cloud_path = session_path / (std::to_string(index) + ".bin");
    if (!SaveCloudBin(cloud_path.string(), session.scan_clouds[index])) {
      return fail("cannot save scan cloud " + std::to_string(index) +
                  " under " + session_path.string());
    }
  }
  return true;
}

}  // namespace serialization
}  // namespace map
}  // namespace atlas
