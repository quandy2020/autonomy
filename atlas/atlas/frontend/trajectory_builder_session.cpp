/*
 * Copyright 2026 The Openbot Authors
 */
#include <mutex>
#include <string>
#include <vector>

#include "backend/global_mapping/pgo_edges.hpp"
#include "frontend/trajectory_builder.hpp"
#include "map/serialization/session_data.hpp"
#include "map/serialization/session_writer.hpp"

namespace atlas {
namespace frontend {

using ::atlas::backend::global_mapping::PgoEdges;
using ::atlas::backend::global_mapping::PgoEdgesIo;
using ::atlas::backend::global_mapping::PreviousSession;
using ::atlas::map::Keyframe;
using ::atlas::map::serialization::SessionData;
using ::atlas::map::serialization::SessionPoseEntry;
using ::atlas::map::serialization::SessionWriter;

using ::atlas::frontend::odometry::ScanPosePtr;

bool TrajectoryBuilder::SaveSession(std::string* error) {
  SessionData session;
  PgoEdges edge_snapshot;
  std::vector<std::string> session_names;
  std::string base_path;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (config_.storage_directory.empty() || scan_poses_.empty()) {
      return true;
    }
    if (config_.session_name.empty()) {
      if (error != nullptr) {
        *error = "voxel/bagname must not be empty when saving a session";
      }
      return false;
    }
    if (session_scan_clouds_.size() != scan_poses_.size()) {
      if (error != nullptr) {
        *error = "scan pose and cloud counts differ; refusing partial save";
      }
      return false;
    }

    session.name = config_.session_name;
    session.poses.reserve(scan_poses_.size());
    session.scan_clouds.reserve(session_scan_clouds_.size());
    for (const ScanPosePtr& scan : scan_poses_) {
      if (!scan) {
        if (error != nullptr) {
          *error = "session contains an invalid scan pose";
        }
        return false;
      }
      SessionPoseEntry entry;
      entry.x = scan->x;
      entry.v6 = scan->v6;
      session.poses.push_back(std::move(entry));
    }
    for (const auto& cloud : session_scan_clouds_) {
      if (!cloud) {
        if (error != nullptr) {
          *error = "session contains an invalid scan cloud";
        }
        return false;
      }
      session.scan_clouds.push_back(*cloud);
    }

    base_path = config_.storage_directory;
    session_names.reserve(previous_sessions_.size() + 1);
    for (const PreviousSession& previous_session : previous_sessions_) {
      session_names.push_back(previous_session.name);
    }
    session_names.push_back(session.name);
    edge_snapshot = loop_closure_edges_;
  }

  if (!SessionWriter().Write(base_path, session, error)) {
    return false;
  }
  if (!PgoEdgesIo(&edge_snapshot, session_names, /*io=*/1, base_path,
                  session.name)) {
    if (error != nullptr) {
      *error = "cannot save PGO edges under " + base_path;
    }
    return false;
  }
  return true;
}

}  // namespace frontend
}  // namespace atlas
