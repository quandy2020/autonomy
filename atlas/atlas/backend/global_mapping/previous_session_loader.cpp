/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/previous_session_loader.hpp"

#include <deque>
#include <filesystem>
#include <iostream>
#include <limits>

#include "frontend/odometry/downsample.hpp"
#include "map/serialization/session_reader.hpp"

namespace atlas {
namespace backend {
namespace global_mapping {

namespace loop_closure = ::atlas::backend::loop_closure;

using ::atlas::frontend::odometry::DownsampleVoxel;
using ::atlas::frontend::odometry::ScanPose;
using ::atlas::map::Keyframe;
using ::atlas::map::serialization::ParsePreviousSessionList;
using ::atlas::map::serialization::PreviousSessionSpec;
using ::atlas::map::serialization::SessionReader;

using ::atlas::frontend::inertial::InertialState;

namespace {

void ClearSessionKeyframes(std::vector<PreviousSession>* sessions) {
  if (sessions == nullptr) {
    return;
  }
  sessions->clear();
}

}  // namespace

bool PreviousSessionLoader::Load(const PreviousSessionOptions& options,
                                 std::vector<PreviousSession>* sessions,
                                 PgoEdges* edges) const {
  if (sessions == nullptr) {
    return false;
  }
  PgoEdges loaded_edges;
  const auto session_specs = ParsePreviousSessionList(options.session_list);
  if (session_specs.empty()) {
    ClearSessionKeyframes(sessions);
    if (edges != nullptr) {
      *edges = PgoEdges{};
    }
    return true;
  }
  std::vector<std::string> session_names;
  for (const PreviousSessionSpec& session_spec : session_specs) {
    session_names.push_back(session_spec.name);
  }
  if (edges != nullptr) {
    const std::filesystem::path edge_path =
        std::filesystem::path(options.base_path) / "edge.txt";
    std::error_code filesystem_error;
    const bool edge_file_exists =
        std::filesystem::exists(edge_path, filesystem_error);
    if (filesystem_error ||
        (edge_file_exists && !PgoEdgesIo(&loaded_edges, session_names, /*io=*/0,
                                         options.base_path, /*bagname=*/""))) {
      return false;
    }
  }

  std::vector<PreviousSession> loaded_sessions;

  const int window_size = std::max(1, options.window_size);
  const int association_size = std::max(1, options.association_size);
  const int mapping_stride = std::max(1, options.mapping_stride);
  const SessionReader session_reader;

  for (const PreviousSessionSpec& session_spec : session_specs) {
    const std::filesystem::path session_name(session_spec.name);
    if (session_name.is_absolute() || session_name.has_parent_path() ||
        session_spec.name == "." || session_spec.name == "..") {
      ClearSessionKeyframes(&loaded_sessions);
      return false;
    }
    PreviousSession loaded_session;
    loaded_session.name = session_spec.name;
    loaded_session.descriptor_score_threshold = session_spec.score_threshold;
    const std::string root =
        (std::filesystem::path(options.base_path) / session_spec.name).string();
    std::vector<SessionPoseEntry> pose_entries;
    std::string read_error;
    if (!session_reader.ReadPoses(options.base_path, session_spec.name,
                                  &pose_entries, &read_error) ||
        pose_entries.size() >
            static_cast<std::size_t>(std::numeric_limits<int>::max())) {
      std::cerr << "PreviousSessionLoader::Load: " << read_error << "\n";
      ClearSessionKeyframes(&loaded_sessions);
      return false;
    }
    loaded_session.scans.reserve(pose_entries.size());
    for (const SessionPoseEntry& entry : pose_entries) {
      auto scan_pose = std::make_shared<ScanPose>(entry.x, nullptr);
      scan_pose->v6 = entry.v6;
      loaded_session.scans.push_back(std::move(scan_pose));
    }
    std::cerr << "PreviousSessionLoader::Load " << root << ": "
              << loaded_session.scans.size() << " scans\n";

    std::deque<sensor::PointCloudXYZI> cloud_window;
    std::deque<InertialState> state_window;
    for (int i = 0; i < static_cast<int>(loaded_session.scans.size()); ++i) {
      sensor::PointCloudXYZI cloud;
      if (!session_reader.ReadCloud(options.base_path, session_spec.name,
                                    static_cast<std::size_t>(i), &cloud,
                                    &read_error)) {
        std::cerr << "PreviousSessionLoader::Load: " << read_error << "\n";
        loaded_sessions.clear();
        return false;
      }
      state_window.push_back(loaded_session.scans[static_cast<size_t>(i)]->x);
      cloud_window.push_back(std::move(cloud));
      if (static_cast<int>(state_window.size()) < window_size) {
        continue;
      }
      InertialState reference_state = state_window.back();
      auto keyframe = std::make_unique<Keyframe>(reference_state);
      keyframe->keyframe_id = i;
      keyframe->session_id = static_cast<int>(loaded_sessions.size());
      keyframe->map_insertion_pending = true;
      sensor::PointCloudXYZI keyframe_cloud;
      for (int j = 0; j < window_size; ++j) {
        const Eigen::Vector3d delta_p =
            reference_state.R.transpose() *
            (state_window[static_cast<size_t>(j)].p - reference_state.p);
        const Eigen::Matrix3d delta_R = reference_state.R.transpose() *
                                        state_window[static_cast<size_t>(j)].R;
        for (const auto& source_point :
             cloud_window[static_cast<size_t>(j)].points) {
          Eigen::Vector3d transformed_point(source_point.x, source_point.y,
                                            source_point.z);
          transformed_point = delta_R * transformed_point + delta_p;
          sensor::PointType pt;
          pt.x = static_cast<float>(transformed_point[0]);
          pt.y = static_cast<float>(transformed_point[1]);
          pt.z = static_cast<float>(transformed_point[2]);
          keyframe_cloud.push_back(pt);
        }
      }
      DownsampleVoxel(&keyframe_cloud, options.voxel_size / 10.0);
      keyframe->point_cloud = std::make_shared<const sensor::PointCloudXYZI>(
          std::move(keyframe_cloud));
      loaded_session.keyframes.push_back(std::move(keyframe));
      for (int j = 0; j < window_size; ++j) {
        cloud_window.pop_front();
        state_window.pop_front();
      }
    }

    loaded_session.loop_detector =
        std::make_unique<loop_closure::triangle::TriangleLoopDetector>(
            options.loop_closure);
    const int subsize = static_cast<int>(loaded_session.keyframes.size());
    for (int i = 0; i + association_size < subsize; i += mapping_stride) {
      const int window_end = i + association_size;
      sensor::PointCloudXYZI descriptor_cloud;
      const InertialState& reference_state =
          loaded_session.keyframes[static_cast<size_t>(window_end - 1)]
              ->inertial_state;
      for (int j = i; j < window_end; ++j) {
        const InertialState& keyframe_state =
            loaded_session.keyframes[static_cast<size_t>(j)]->inertial_state;
        const Eigen::Vector3d delta_p = reference_state.R.transpose() *
                                        (keyframe_state.p - reference_state.p);
        const Eigen::Matrix3d delta_R =
            reference_state.R.transpose() * keyframe_state.R;
        for (const auto& source_point :
             loaded_session.keyframes[static_cast<size_t>(j)]
                 ->point_cloud->points) {
          Eigen::Vector3d transformed_point(source_point.x, source_point.y,
                                            source_point.z);
          transformed_point = delta_R * transformed_point + delta_p;
          sensor::PointType transformed_sensor_point;
          transformed_sensor_point.x = static_cast<float>(transformed_point[0]);
          transformed_sensor_point.y = static_cast<float>(transformed_point[1]);
          transformed_sensor_point.z = static_cast<float>(transformed_point[2]);
          descriptor_cloud.push_back(transformed_sensor_point);
        }
      }
      std::vector<loop_closure::triangle::TriangleDescriptor> descriptors;
      sensor::PointCloudXYZI plane;
      loaded_session.loop_detector->GenerateDescriptors(
          descriptor_cloud,
          loaded_session.keyframes[static_cast<size_t>(window_end - 1)]
              ->keyframe_id,
          &descriptors, &plane);
      loaded_session.loop_detector->AddDescriptors(descriptors, plane);
    }
    loaded_session.loop_detector->DisableNearSkip();
    loaded_sessions.push_back(std::move(loaded_session));
    std::cerr << "PreviousSessionLoader::Load done: " << root
              << " keyframes=" << loaded_sessions.back().keyframes.size()
              << " btc="
              << loaded_sessions.back().loop_detector->database_size() << "\n";
  }
  if (loaded_sessions.empty()) {
    return false;
  }
  ClearSessionKeyframes(sessions);
  *sessions = std::move(loaded_sessions);
  if (edges != nullptr) {
    *edges = std::move(loaded_edges);
  }
  return true;
}

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas
