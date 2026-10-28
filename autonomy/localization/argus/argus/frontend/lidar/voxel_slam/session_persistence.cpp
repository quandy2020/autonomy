/*
 * Copyright 2026 The Openbot Authors
 */
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "backend/global_mapping/pgo_edges.hpp"
#include "frontend/lidar/voxel_slam/voxel_slam.hpp"
#include "map/voxel/serialization/voxel_map_serializer.hpp"

namespace argus {
namespace vxslam {

bool Odometry::SaveSession(std::string* error) {
  const auto fail = [error](const std::string& message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (config_.save_path.empty() || scan_poses_.empty()) {
    return true;
  }
  if (config_.bagname.empty()) {
    return fail("voxel/bagname must not be empty when saving a session");
  }

  namespace fs = std::filesystem;
  const fs::path session_name(config_.bagname);
  if (session_name.is_absolute() || session_name.has_parent_path() ||
      config_.bagname == "." || config_.bagname == "..") {
    return fail("voxel/bagname must be a single directory name");
  }
  const fs::path root = fs::path(config_.save_path) / session_name;
  std::error_code ec;
  fs::create_directories(root, ec);
  if (ec) {
    return fail("cannot create session directory " + root.string() + ": " +
                ec.message());
  }

  std::vector<SessionPoseEntry> poses;
  poses.reserve(scan_poses_.size());
  for (const ScanPosePtr& scan : scan_poses_) {
    if (!scan || !std::isfinite(scan->x.t) || !scan->x.p.allFinite() ||
        !scan->x.R.allFinite() || !scan->x.v.allFinite() ||
        !scan->x.bg.allFinite() || !scan->x.ba.allFinite() ||
        !scan->x.g.allFinite() || !scan->v6.allFinite()) {
      return fail("session contains an invalid scan pose");
    }
    SessionPoseEntry entry;
    entry.x = scan->x;
    entry.v6 = scan->v6;
    poses.push_back(entry);
  }
  if (keyframes_.size() != scan_poses_.size()) {
    return fail("scan pose and keyframe counts differ; refusing partial save");
  }
  if (!SaveSessionPoses((root / "alidarState.txt").string(), poses)) {
    return fail("cannot save session poses under " + root.string());
  }
  for (size_t i = 0; i < keyframes_.size(); ++i) {
    const Keyframe* keyframe = keyframes_[i];
    if (keyframe == nullptr || !keyframe->plptr ||
        std::any_of(
            keyframe->plptr->points.begin(), keyframe->plptr->points.end(),
            [](const PointType& point) {
              return !std::isfinite(point.x) || !std::isfinite(point.y) ||
                     !std::isfinite(point.z) || !std::isfinite(point.intensity);
            }) ||
        !SaveCloudBin((root / (std::to_string(i) + ".bin")).string(),
                      *keyframe->plptr)) {
      return fail("cannot save keyframe cloud " + std::to_string(i) +
                  " under " + root.string());
    }
  }

  std::vector<std::string> session_names;
  session_names.reserve(prev_sessions_.size() + 1);
  for (const PreviousMapSession& session : prev_sessions_) {
    session_names.push_back(session.name);
  }
  session_names.push_back(config_.bagname);
  if (!PgoEdgesIo(&lp_edges_, session_names, /*io=*/1,
                  fs::path(config_.save_path).string(), config_.bagname)) {
    return fail("cannot save PGO edges under " + config_.save_path);
  }
  return true;
}

}  // namespace vxslam
}  // namespace argus
