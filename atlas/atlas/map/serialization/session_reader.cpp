/*
 * Copyright 2026 The Openbot Authors
 */
#include "map/serialization/session_reader.hpp"

#include <filesystem>

namespace atlas {
namespace map {
namespace serialization {
namespace {

bool SessionPath(const std::string& base_path, const std::string& session_name,
                 std::filesystem::path* session_path, std::string* error) {
  const auto fail = [error](const std::string& message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };
  if (session_path == nullptr) {
    return fail("session path output must not be null");
  }
  const std::filesystem::path name(session_name);
  if (session_name.empty() || name.is_absolute() || name.has_parent_path() ||
      session_name == "." || session_name == "..") {
    return fail("session name must be a single directory name");
  }
  *session_path = std::filesystem::path(base_path) / name;
  return true;
}

}  // namespace

bool SessionReader::ReadPoses(const std::string& base_path,
                              const std::string& session_name,
                              std::vector<SessionPoseEntry>* poses,
                              std::string* error) const {
  if (poses == nullptr) {
    if (error != nullptr) {
      *error = "pose output must not be null";
    }
    return false;
  }
  std::filesystem::path session_path;
  if (!SessionPath(base_path, session_name, &session_path, error)) {
    return false;
  }
  std::vector<SessionPoseEntry> loaded_poses;
  const std::filesystem::path pose_path = session_path / "alidarState.txt";
  if (!LoadSessionPoses(pose_path.string(), &loaded_poses)) {
    if (error != nullptr) {
      *error = "cannot read session poses from " + pose_path.string();
    }
    return false;
  }
  *poses = std::move(loaded_poses);
  return true;
}

bool SessionReader::ReadCloud(const std::string& base_path,
                              const std::string& session_name,
                              std::size_t scan_index,
                              sensor::PointCloudXYZI* cloud,
                              std::string* error) const {
  if (cloud == nullptr) {
    if (error != nullptr) {
      *error = "cloud output must not be null";
    }
    return false;
  }
  std::filesystem::path session_path;
  if (!SessionPath(base_path, session_name, &session_path, error)) {
    return false;
  }
  const std::filesystem::path cloud_path =
      session_path / (std::to_string(scan_index) + ".bin");
  sensor::PointCloudXYZI loaded_cloud;
  if (!LoadCloudBin(cloud_path.string(), &loaded_cloud)) {
    if (error != nullptr) {
      *error = "cannot read scan cloud from " + cloud_path.string();
    }
    return false;
  }
  *cloud = std::move(loaded_cloud);
  return true;
}

}  // namespace serialization
}  // namespace map
}  // namespace atlas
